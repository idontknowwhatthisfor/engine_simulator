#!/usr/bin/env python3
"""Split the Sketchfab inline-4 glTF into rigid motion groups and write engine.mesh.

Standard library only. Usage:
    python3 -I engines/inline4_fit/extract_parts.py [model_dir] [out_file]
    (defaults: this folder's scene.gltf / scene.bin -> engine.mesh beside them)

The glTF merges every part into one mesh per material, so each mesh is split into
connected triangle islands, every island is classified (static / crank / piston / rod)
and assigned to a cylinder, then islands are merged back into one draw group per
(kind, cylinder, material). Pivots (crankpins, gudgeon pins) are measured from the
geometry, nothing is hard-coded except the classification rules.

All coordinates are raw mesh space (mm): crank axis = Z, cylinders point toward -Y.
The root node matrix (Y and Z flipped) is stored in the file for the renderer.

engine.mesh is the ENGMSHv3 engine format (see tools/meshlib.py). Per cylinder: z_centre, gudgeon
(x, y), crankpin (x, y), throw_r, theta0 (rad, crankpin angle from the bore axis, toward +X), rod_L,
then bank 0 (the model's bores all point toward -Y), the deck (the lowest point of the see-through
block), the 85 mm bore and the firing lag from FIRING. Metadata: banked = 0 (the original model, its
skirts sit between the counterweights). Files written before v3 have the magic "I4ENGv1" and only
the first eight values per cylinder; the simulator still reads them.
"""
import json
import math
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "tools"))
from meshlib import Mesh, write_engine  # noqa: E402

KIND_NAMES = ["static", "crank", "piston", "rod", "block"]
FIRING = (1, 3, 4, 2)   # flat-plane crank, cylinders numbered from the timing end
BORE = 85.0
STATIC, CRANK, PISTON, ROD, BLOCK = range(5)

# mesh index -> what it holds (see plan.md section 1)
MESH_BLOCK, MESH_BRONZE, MESH_SHELLS, MESH_CRANK, MESH_RECIP, MESH_PINS, MESH_CAPS, MESH_CAPBOLTS = range(8)


def load(model_dir):
    with open(os.path.join(model_dir, "scene.gltf")) as f:
        g = json.load(f)
    with open(os.path.join(model_dir, "scene.bin"), "rb") as f:
        buf = f.read()
    return g, buf


def read_accessor(g, buf, idx):
    a = g["accessors"][idx]
    bv = g["bufferViews"][a["bufferView"]]
    ncomp = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}[a["type"]]
    fmt = {5126: "f", 5125: "I", 5123: "H", 5121: "B"}[a["componentType"]]
    size = struct.calcsize(fmt) * ncomp
    stride = bv.get("byteStride", size)
    off = bv.get("byteOffset", 0) + a.get("byteOffset", 0)
    if stride == size:
        flat = struct.unpack_from("<" + fmt * (ncomp * a["count"]), buf, off)
        if ncomp == 1:
            return list(flat)
        return [flat[k:k + ncomp] for k in range(0, len(flat), ncomp)]
    out = [struct.unpack_from("<" + fmt * ncomp, buf, off + k * stride) for k in range(a["count"])]
    return [v[0] for v in out] if ncomp == 1 else out


def components(pos, idx):
    """Connected triangle islands, vertices welded by position. Returns list of triangle-index lists."""
    weld = {}
    rep = [weld.setdefault((round(p[0], 3), round(p[1], 3), round(p[2], 3)), len(weld)) for p in pos]
    parent = list(range(len(weld)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for t in range(0, len(idx), 3):
        a, b, c = find(rep[idx[t]]), find(rep[idx[t + 1]]), find(rep[idx[t + 2]])
        parent[b] = a
        parent[find(c)] = a
    tris = {}
    for t in range(0, len(idx), 3):
        tris.setdefault(find(rep[idx[t]]), []).append(t // 3)
    return list(tris.values())


class Island:
    def __init__(self, mesh, pos, idx, tri_list):
        self.mesh = mesh
        verts = sorted({idx[3 * t + k] for t in tri_list for k in range(3)})
        self.verts = verts
        self.tris = tri_list
        xs = [pos[v][0] for v in verts]
        ys = [pos[v][1] for v in verts]
        zs = [pos[v][2] for v in verts]
        self.lo = (min(xs), min(ys), min(zs))
        self.hi = (max(xs), max(ys), max(zs))
        self.centre = tuple((self.lo[k] + self.hi[k]) / 2 for k in range(3))
        self.size = tuple(self.hi[k] - self.lo[k] for k in range(3))
        self.kind = STATIC
        self.cyl = -1

    def __repr__(self):
        return "mesh %d n=%d c=(%.1f, %.1f, %.1f) size=(%.1f, %.1f, %.1f)" % (
            (self.mesh, len(self.verts)) + self.centre + self.size)


def circle_fit(pts):
    """Kasa least-squares circle fit, returns (cx, cy, radius, rms error)."""
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    suu = suv = svv = suuu = svvv = suvv = svuu = 0.0
    for x, y in pts:
        u, v = x - mx, y - my
        suu += u * u; suv += u * v; svv += v * v
        suuu += u ** 3; svvv += v ** 3; suvv += u * v * v; svuu += v * u * u
    det = suu * svv - suv * suv
    b0, b1 = (suuu + suvv) / 2, (svvv + svuu) / 2
    uc = (b0 * svv - b1 * suv) / det
    vc = (suu * b1 - suv * b0) / det
    cx, cy = uc + mx, vc + my
    r = math.sqrt(uc * uc + vc * vc + (suu + svv) / n)
    rms = math.sqrt(sum((math.hypot(x - cx, y - cy) - r) ** 2 for x, y in pts) / n)
    return cx, cy, r, rms


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    model_dir = sys.argv[1] if len(sys.argv) > 1 else here  # scene.gltf / scene.bin sit beside this script
    out_file = sys.argv[2] if len(sys.argv) > 2 else os.path.join(model_dir, "engine.mesh")
    g, buf = load(model_dir)

    root = g["nodes"][0].get("matrix", [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])
    # the parts sit directly under node 1, which must carry no transform of its own
    for n in g["nodes"][1:]:
        assert not any(k in n for k in ("matrix", "translation", "rotation", "scale")), n["name"]

    mesh_of_node = [g["nodes"][c]["mesh"] for c in g["nodes"][1]["children"]]
    assert mesh_of_node == list(range(8)), mesh_of_node

    meshes = []  # (material, positions, normals, indices)
    islands = []
    for mi, m in enumerate(g["meshes"]):
        assert len(m["primitives"]) == 1
        p = m["primitives"][0]
        pos = read_accessor(g, buf, p["attributes"]["POSITION"])
        nor = read_accessor(g, buf, p["attributes"]["NORMAL"])
        idx = read_accessor(g, buf, p["indices"])
        meshes.append((p["material"], pos, nor, idx))
        islands += [Island(mi, pos, idx, t) for t in components(pos, idx)]

    # ---- classify ------------------------------------------------------------------
    pistons = [i for i in islands if i.mesh == MESH_RECIP and i.size[0] > 70 and i.size[2] > 70]
    pistons.sort(key=lambda i: i.centre[2])
    assert len(pistons) == 4, "expected 4 pistons, found %d" % len(pistons)
    cyl_z = [p.centre[2] for p in pistons]
    bore = sum(p.size[0] for p in pistons) / 4

    def nearest_cyl(z):
        return min(range(4), key=lambda c: abs(cyl_z[c] - z))

    for i in islands:
        if i.mesh == MESH_BLOCK:
            i.kind = BLOCK
        elif i.mesh == MESH_CRANK:
            i.kind = CRANK
        elif i.mesh == MESH_PINS:
            i.kind = PISTON
        elif i.mesh == MESH_RECIP:
            # pistons; everything else in this material is a rod body, cap bolt or nut
            i.kind = PISTON if i in pistons else ROD
        elif i.mesh == MESH_SHELLS:
            # bearing shells hug the crank (|y| < 100); off-axis ones ride on the crankpins
            r_axis = math.hypot(i.centre[0], i.centre[1])
            if abs(i.centre[1]) < 100 and r_axis > 30:
                i.kind = ROD
            elif abs(i.centre[1]) >= 100:
                i.kind = BLOCK  # freeze plugs, hidden together with the block
        if i.kind in (PISTON, ROD):
            i.cyl = nearest_cyl(i.centre[2])

    # ---- measure pivots ------------------------------------------------------------
    crank_pos = meshes[MESH_CRANK][1]
    shells_pos = meshes[MESH_SHELLS][1]

    # crank axis from the crank's own main journals: crank vertices inside each main bearing
    # shell's z range that lie close to the axis (the shells themselves sit ~0.07 mm off)
    mains = [i for i in islands if i.mesh == MESH_SHELLS and i.kind == STATIC and abs(i.centre[1]) < 100]
    assert len(mains) == 5, mains
    main_fits = []
    for i in mains:
        pts = [(x, y) for x, y, z in crank_pos
               if i.lo[2] - 0.5 <= z <= i.hi[2] + 0.5 and math.hypot(x - i.centre[0], y - i.centre[1]) < 25]
        main_fits.append(circle_fit(pts))
    axis_x = sum(f[0] for f in main_fits) / len(main_fits)
    axis_y = sum(f[1] for f in main_fits) / len(main_fits)
    print("main journals (x, y, r, rms):", ", ".join("(%.3f, %.3f, %.2f, %.3f)" % f for f in main_fits))

    cylinders = []
    report = []
    for c in range(4):
        pin = [i for i in islands if i.mesh == MESH_PINS and i.cyl == c]
        shell = [i for i in islands if i.mesh == MESH_SHELLS and i.kind == ROD and i.cyl == c]
        assert len(pin) == 1 and len(shell) == 1, (c, pin, shell)
        pin, shell = pin[0], shell[0]
        gx, gy = pin.centre[0], pin.centre[1]

        # crankpin: fit a circle to crank vertices inside the big-end shell's z range, near the shell
        sx, sy, sr, _ = circle_fit([(shells_pos[v][0], shells_pos[v][1]) for v in shell.verts])
        z0, z1 = shell.lo[2], shell.hi[2]
        pts = [(x, y) for x, y, z in crank_pos
               if z0 - 0.5 <= z <= z1 + 0.5 and math.hypot(x - sx, y - sy) < sr + 2]
        cx, cy, cr, rms = circle_fit(pts)

        dx, dy = cx - axis_x, cy - axis_y
        throw = math.hypot(dx, dy)
        # bore axis points toward -Y; theta0 measured from it, positive toward +X
        theta0 = math.atan2(dx, -dy)
        rod_l = math.hypot(cx - gx, cy - gy)
        # slider-crank consistency: pin distance from the axis along the bore
        s_pred = throw * math.cos(theta0) + math.sqrt(rod_l ** 2 - (throw * math.sin(theta0)) ** 2)
        s_mesh = axis_y - gy
        cylinders.append((cyl_z[c], gx, gy, cx, cy, throw, theta0, rod_l))
        report.append((c + 1, cyl_z[c], gx, gy, cx, cy, cr, rms, len(pts), sx - cx, sy - cy,
                       throw, math.degrees(theta0), rod_l, s_pred - s_mesh))

    # ---- merge islands into draw groups ----------------------------------------------
    groups = {}
    for i in islands:
        mat = meshes[i.mesh][0]
        groups.setdefault((i.kind, i.cyl, mat), []).append(i)

    # ---- report ----------------------------------------------------------------------
    print("islands: %d   bore: %.1f mm   crank axis: (%.2f, %.2f)" % (len(islands), bore, axis_x, axis_y))
    print("\nisland classification per mesh:")
    for mi in range(8):
        counts = {}
        for i in islands:
            if i.mesh == mi:
                counts[KIND_NAMES[i.kind]] = counts.get(KIND_NAMES[i.kind], 0) + 1
        print("  mesh %d: %s" % (mi, ", ".join("%s %d" % kv for kv in sorted(counts.items()))))

    print("\nper-cylinder parts (islands): piston-group / rod-group")
    for c in range(4):
        pg = [i for i in islands if i.kind == PISTON and i.cyl == c]
        rg = [i for i in islands if i.kind == ROD and i.cyl == c]
        print("  cyl %d: piston %d (pistons %d, pins %d)   rod %d (body+fasteners %d, big-end shell %d)" % (
            c + 1, len(pg), sum(i.mesh == MESH_RECIP for i in pg), sum(i.mesh == MESH_PINS for i in pg),
            len(rg), sum(i.mesh == MESH_RECIP for i in rg), sum(i.mesh == MESH_SHELLS for i in rg)))

    print("\nkinematics (mm, deg):")
    print("  cyl  z_c     gudgeon(x,y)      crankpin(x,y)    pinR   fitRMS  npts  shell-off   throw  theta0   rod_L  s_err")
    for r in report:
        print("  %d  %7.1f  (%6.2f,%8.2f)  (%7.2f,%7.2f)  %5.2f  %6.3f  %4d  (%4.2f,%4.2f)  %6.2f  %6.2f  %6.2f  %6.3f" % r)

    print("\ndraw groups: %d" % len(groups))
    for (kind, cyl, mat), isl in sorted(groups.items()):
        print("  %-6s cyl %2s  mat %d  islands %3d  verts %6d  tris %6d" % (
            KIND_NAMES[kind], cyl + 1 if cyl >= 0 else "-", mat, len(isl),
            sum(len(i.verts) for i in isl), sum(len(i.tris) for i in isl)))

    # ---- write -------------------------------------------------------------------------
    mats = []
    for m in g["materials"]:
        pbr = m.get("pbrMetallicRoughness", {})
        rgba = pbr.get("baseColorFactor", [1, 1, 1, 1])
        mats.append((*rgba, pbr.get("metallicFactor", 1.0), pbr.get("roughnessFactor", 1.0),
                     1 if m.get("alphaMode") == "BLEND" else 0))
    out_groups = []
    for (kind, cyl, mat), isl in sorted(groups.items()):
        mesh, remap = Mesh(), {}
        for i in isl:
            _, pos, nor, idx = meshes[i.mesh]
            for t in i.tris:
                for k in range(3):
                    v = idx[3 * t + k]
                    if v not in remap:
                        remap[v] = len(remap)
                        mesh.v += pos[v]
                        mesh.v += nor[v]
                    mesh.i.append(remap[v])
        out_groups.append((kind, cyl, mat, mesh))
    # the deck is the lowest point (mesh -y is up) of the see-through block, as stored in f32
    f32 = lambda x: struct.unpack("<f", struct.pack("<f", x))[0]  # noqa: E731
    deck = min(f32(y) for kind, _, mat, mesh in out_groups if kind == BLOCK and mats[mat][6]
               for y in mesh.v[1::6])
    lag = [0.0] * len(cylinders)
    for k, cn in enumerate(FIRING):
        lag[cn - 1] = 720.0 / len(FIRING) * k
    cyls = [tuple(cyl) + (0.0, deck, BORE, lag[c]) for c, cyl in enumerate(cylinders)]
    size = write_engine(out_file, root, mats, cyls, out_groups, [("banked", [0.0])], (axis_x, axis_y, 0.0))
    print("\nwrote %s (%.1f MB)" % (out_file, size / 1e6))

if __name__ == "__main__":
    main()
