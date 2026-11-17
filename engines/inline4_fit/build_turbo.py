#!/usr/bin/env python3
"""Split the Sketchfab "Car Turbocharger" glTF into housing / rotor / bearing and write turbo.mesh.

Standard library only. Usage:
    python3 -I engines/inline4_fit/build_turbo.py [model_dir] [out_file]
    (defaults: turbocharger/ beside this file -> turbo.mesh beside this file)

The model has one mesh per material, and the "metal" one holds both the housings and the shaft, so
each mesh is split into connected triangle islands and every island is sorted by geometry:
  - rotor: within RIN of the shaft axis (compressor wheel, turbine wheel, shaft, nuts, wheel backs)
  - bearing: within RIN but filling the centre housing (it stays still while the rotor spins)
  - housing: everything else (compressor, centre and turbine housings, flanges, clamps, bolts),
    plus the flat inner walls of the centre housing; the simulator draws it see-through
The rule only looks at the geometry, so it gives the same split on the original glTF and on the
copy whose housings were moved to a see-through "housing_ghost" material.

Output frame (mm, scaled by the model's own 0.737): the shaft is the +x axis, x = 0 at the turbine
outlet face, +x toward the compressor; y and z are the model's raw axes.

turbo.mesh layout (little endian):
    char[8]  magic "TURBOv1\\0"
    u32      group count, then per group:
                 u32 kind (0 housing, 1 rotor, 2 bearing), f32 rgba[4], f32 roughness, f32 metallic,
                 u32 vertex count, u32 index count, f32[6 * vertex count] (px py pz nx ny nz),
                 u32[index count]
"""
import json
import math
import os
import struct
import sys

HOUSING, ROTOR, BEARING = range(3)
AXIS_Y, AXIS_Z, X0 = 0.624, 0.946, -0.424  # shaft axis (along raw x) and the turbine outlet face
RIN = 0.052                                 # radius of the rotating assembly
SCALE = 0.7373252511024475 * 1000.0         # the model's root scale, metres -> mm

# per original material: rgba (linear), roughness, metallic
LOOK = {
    "copper": ((0.72, 0.42, 0.20, 1.0), 0.35, 0.9),
    "metal": ((0.66, 0.66, 0.68, 1.0), 0.3, 0.85),
    "metal-rough": ((0.50, 0.50, 0.52, 1.0), 0.6, 0.6),
    "metalblack": ((0.10, 0.10, 0.11, 1.0), 0.5, 0.3),
}
GHOST = ((0.3662526, 0.3564001, 0.3324515, 0.25), 0.65, 0.0)  # the inline-4 block's see-through material


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "turbocharger")
    out_file = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "turbo.mesh")
    g = json.load(open(os.path.join(src, "scene.gltf")))
    blob = open(os.path.join(src, g["buffers"][0].get("uri", "scene.bin")), "rb").read()

    def acc(i):
        a = g["accessors"][i]
        v = g["bufferViews"][a["bufferView"]]
        off = v.get("byteOffset", 0) + a.get("byteOffset", 0)
        n = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}[a["type"]]
        fmt = {5126: "f", 5125: "I", 5123: "H"}[a["componentType"]]
        size = struct.calcsize(fmt)
        stride = v.get("byteStride", n * size)
        if stride == n * size:
            flat = struct.unpack_from("<%d%s" % (a["count"] * n, fmt), blob, off)
            return [flat[k * n:(k + 1) * n] for k in range(a["count"])]
        return [struct.unpack_from("<" + fmt * n, blob, off + k * stride) for k in range(a["count"])]

    # (kind, look) -> list of (positions, normals, triangle index list)
    groups = {}
    counts = {HOUSING: 0, ROTOR: 0, BEARING: 0}
    for mesh in g["meshes"]:
        for prim in mesh["primitives"]:
            name = g["materials"][prim["material"]].get("name", "")
            look = LOOK.get(name, LOOK["metal"])
            P = acc(prim["attributes"]["POSITION"])
            N = acc(prim["attributes"]["NORMAL"])
            I = [x[0] for x in acc(prim["indices"])]
            # islands: triangles joined by shared (welded) positions
            key = {}
            rep = [key.setdefault((round(p[0], 6), round(p[1], 6), round(p[2], 6)), k) for k, p in enumerate(P)]
            parent = list(range(len(P)))

            def find(x):
                while parent[x] != x:
                    parent[x] = parent[parent[x]]
                    x = parent[x]
                return x

            for t in range(0, len(I), 3):
                a, b, c = find(rep[I[t]]), find(rep[I[t + 1]]), find(rep[I[t + 2]])
                parent[a] = b
                parent[find(c)] = find(b)
            islands = {}
            for t in range(0, len(I), 3):
                islands.setdefault(find(rep[I[t]]), []).append(t // 3)
            for tris in islands.values():
                vs = [P[I[3 * t + j]] for t in tris for j in range(3)]
                r = max(math.hypot(v[1] - AXIS_Y, v[2] - AXIS_Z) for v in vs)
                x0, x1 = min(v[0] for v in vs), max(v[0] for v in vs)
                if r > RIN or (x1 - x0 < 0.001 and -0.317 <= x0 <= -0.262):
                    kind = HOUSING
                elif -0.32 < x0 < -0.30 and -0.27 < x1 < -0.25:  # inside the centre housing only
                    kind = BEARING
                else:
                    kind = ROTOR
                counts[kind] += len(tris)
                gl = GHOST if kind == HOUSING else look
                groups.setdefault((kind, gl), []).append((P, N, [I[3 * t + j] for t in tris for j in range(3)]))

    out = bytearray(b"TURBOv1\0")
    out += struct.pack("<I", len(groups))
    for (kind, (rgba, rough, metal)), parts in sorted(groups.items(), key=lambda kv: (kv[0][0], kv[0][1][0])):
        vdata, idata = [], []
        for P, N, idx in parts:
            remap = {}
            for v in idx:
                if v not in remap:
                    remap[v] = len(vdata) // 6
                    p, n = P[v], N[v]
                    vdata += [(p[0] - X0) * SCALE, (p[1] - AXIS_Y) * SCALE, (p[2] - AXIS_Z) * SCALE, n[0], n[1], n[2]]
                idata.append(remap[v])
        out += struct.pack("<I4f2fII", kind, *rgba, rough, metal, len(vdata) // 6, len(idata))
        out += struct.pack("<%df" % len(vdata), *vdata)
        out += struct.pack("<%dI" % len(idata), *idata)
    with open(out_file, "wb") as f:
        f.write(out)
    print("turbo: housing %d, rotor %d, bearing %d triangles in %d groups" %
          (counts[HOUSING], counts[ROTOR], counts[BEARING], len(groups)))
    print("wrote %s (%.1f MB)" % (out_file, len(out) / 1e6))


if __name__ == "__main__":
    main()
