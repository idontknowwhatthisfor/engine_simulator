#!/usr/bin/env python3
"""Build stats.md: every engine and version (engine_sim --engine-type list --ids), its specs, dyno
peaks and a drag run (engine_sim --drag-stats, src/drag_stats.hpp). Run from the project root:

    python3 scripts/gen_stats.py [--bin ./engine_sim] [--out stats.md] [-j N]
"""
import argparse
import os
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

KMH_TO_MPH = 0.621371
KW_TO_HP = 1.341022
NM_TO_LBFT = 0.737562


def run_one(binary, cfg):
    out = subprocess.run([binary, "--engine-type", *cfg.split(), "--drag-stats"],
                         capture_output=True, text=True, check=True).stdout
    d = {}
    for line in out.splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            try:
                d[k] = float(v)
            except ValueError:
                d[k] = v
    d["cfg"] = cfg
    d["id"] = cfg.split()[0]
    d["variant"] = cfg.split()[2] if "--variant" in cfg else "stock"
    return d


def t(v):
    return "—" if v is None or v < 0 else f"{v:.2f} s"


def hp(d):
    return d["peak_kw"] * KW_TO_HP


def power(d):
    return f"{hp(d):.0f} hp ({d['peak_kw']:.0f} kW) @ {d['peak_kw_rpm']:.0f}"


def torque(d):
    return f"{d['peak_nm']:.0f} N·m ({d['peak_nm'] * NM_TO_LBFT:.0f} lb·ft) @ {d['peak_nm_rpm']:.0f}"


def speed(kmh):
    return "—" if kmh <= 0 else f"{kmh:.0f} km/h ({kmh * KMH_TO_MPH:.0f} mph)"


def run_time(d, key):
    """an ET with its trap speed"""
    s = d[f"{key}_s"]
    return "—" if s < 0 else f"{s:.2f} s @ {d[f'{key}_kmh']:.0f} km/h ({d[f'{key}_kmh'] * KMH_TO_MPH:.0f} mph)"


def ordinal(n):
    n = int(n)
    return f"{n}{'th' if 10 <= n % 100 <= 20 else {1: 'st', 2: 'nd', 3: 'rd'}.get(n % 10, 'th')}"


def top_how(d):
    """the gear, the rpm and what holds it there"""
    held = "on the limiter" if d["top_limit"] == "limiter" else "drag-limited"
    return f"{ordinal(d['top_gear'])} gear, {held} at {d['top_rpm']:.0f} rpm"


def label(d):
    """the engine's name; a version's name already says which version it is"""
    return d["name"]


def delta(a, b, key, unit, fmt="{:+.0f}", better_low=False):
    va, vb = a[key], b[key]
    if va is None or vb is None or va < 0 or vb < 0:
        return ""
    return fmt.format(vb - va) + unit


ROWS = [  # (row label, formatter, delta key, unit, delta format)
    ("Engine", lambda d: f"{d['cc']:.0f} cc, {d['cylinders']:.0f} cyl, {d['boost'] if d['boost'] != 'none' else 'naturally aspirated'}", None, "", ""),
    ("Peak power", power, "hp", " hp", "{:+.0f}"),
    ("Peak torque", torque, "peak_nm", " N·m", "{:+.0f}"),
    ("Rev limit", lambda d: f"{d['rev_limit']:.0f} rpm", "rev_limit", " rpm", "{:+.0f}"),
    ("Mass (with rider for bikes)", lambda d: f"{d['mass_kg']:.0f} kg", "mass_kg", " kg", "{:+.0f}"),
    ("Power to weight", lambda d: f"{hp(d) / d['mass_kg'] * 1000:.0f} hp/t", "hpt", " hp/t", "{:+.0f}"),
    ("Gearbox", lambda d: f"{d['gears']:.0f}-speed, final {d['final_drive']:.2f}", None, "", ""),
    ("Launch control held", lambda d: f"{d['launch_rpm']:.0f} rpm" + (f", {d['launch_boost_bar']:+.2f} bar" if d['boost'] != 'none' else ""), None, "", ""),
    ("0–60 mph", lambda d: t(d["t_60mph"]), "t_60mph", " s", "{:+.2f}"),
    ("0–100 km/h", lambda d: t(d["t_100kmh"]), "t_100kmh", " s", "{:+.2f}"),
    ("0–200 km/h", lambda d: t(d["t_200kmh"]), "t_200kmh", " s", "{:+.2f}"),
    ("0–300 km/h", lambda d: t(d["t_300kmh"]), "t_300kmh", " s", "{:+.2f}"),
    ("⅛ mile", lambda d: run_time(d, "eighth"), "eighth_s", " s", "{:+.2f}"),
    ("¼ mile", lambda d: run_time(d, "quarter"), "quarter_s", " s", "{:+.2f}"),
    ("1 km", lambda d: run_time(d, "km"), "km_s", " s", "{:+.2f}"),
    ("Top speed", lambda d: speed(d["top_kmh"]) + f", {top_how(d)}", "top_kmh", " km/h", "{:+.0f}"),
    ("Peak acceleration", lambda d: f"{d['peak_g']:.2f} g", "peak_g", " g", "{:+.2f}"),
]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bin", default="./engine_sim")
    ap.add_argument("--out", default="stats.md")
    ap.add_argument("-j", type=int, default=os.cpu_count() or 4)
    a = ap.parse_args()

    cfgs = subprocess.run([a.bin, "--engine-type", "list", "--ids"], capture_output=True, text=True,
                          check=True).stdout.split("\n")
    cfgs = [c.strip() for c in cfgs if c.strip()]
    print(f"running {len(cfgs)} configurations ({a.j} at a time)...", file=sys.stderr)
    with ThreadPoolExecutor(a.j) as pool:
        results = list(pool.map(lambda c: run_one(a.bin, c), cfgs))
    for d in results:
        d["hp"] = hp(d)
        d["hpt"] = hp(d) / d["mass_kg"] * 1000

    try:
        commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip()
    except OSError:
        commit = "?"

    by_id = {}
    for d in results:
        by_id.setdefault(d["id"], []).append(d)

    L = []
    L.append("# Vehicle stats\n")
    L.append(f"_Generated by `scripts/gen_stats.py` (`make stats`) from `engine_sim --drag-stats` at commit "
             f"`{commit}`. Do not edit by hand._\n")
    L.append("## How the numbers are measured\n")
    L.append("- **Power and torque**: the full-throttle dyno sweep (`--dyno`), 1000 rpm to the limiter in "
             "250 rpm (cars) or 500 rpm (bikes) steps, so the peaks are to the nearest step.")
    L.append("- **Drag run**: every vehicle the same way. Started and idling in the automatic's normal mode, "
             "on a flat road at sea level (air 1.2 kg/m³). **Launch control**: brake and full throttle together "
             "for 3 s, then the brake is let go and the throttle held flat for 60 s. Times start when the brake is released.")
    L.append("- **Traction is unlimited**: the simulator has no tyre grip limit, and bikes run with **wheelie "
             "control off**, so launches are better than a real car or bike could put down.")
    L.append("- **Top speed** is the highest speed the vehicle can hold flat out, in its best gear (not "
             "necessarily the gear the automatic would pick). For each gear it is the speed where the drive force at "
             "the wheels (the engine's full-throttle torque measured at that rpm and road speed) meets rolling and "
             "air drag. If the engine still has force to spare at the rev limiter, the vehicle is held in that gear "
             "in the full simulation until its speed settles on the limiter, and that average is used. It is the "
             "settled speed, however long the vehicle takes to reach it, so it is not cut short by a time limit.")
    L.append("- ⅛ mile = 201.168 m, ¼ mile = 402.336 m. Trap speed is the speed at the line. "
             "— means not reached within the 60 s.\n")

    # summary
    L.append("## Summary\n")
    L.append("| Vehicle | Version | Power | Torque | Mass | hp/t | 0–100 km/h | ¼ mile | Top speed |")
    L.append("|---|---|---|---|---|---|---|---|---|")
    for vid, ds in by_id.items():
        for d in ds:
            L.append(f"| {d['name']} | {d['variant']} | {d['hp']:.0f} hp | {d['peak_nm']:.0f} N·m | {d['mass_kg']:.0f} kg | "
                     f"{d['hpt']:.0f} | {t(d['t_100kmh'])} | {run_time(d, 'quarter')} | {speed(d['top_kmh'])} |")
    L.append("")

    # drag board
    L.append("## Drag board (¼ mile)\n")
    L.append("| # | Vehicle | Type | ¼ mile | Trap speed | 0–100 km/h |")
    L.append("|---|---|---|---|---|---|")
    ranked = sorted(results, key=lambda d: d["quarter_s"] if d["quarter_s"] > 0 else 1e9)
    for k, d in enumerate(ranked, 1):
        L.append(f"| {k} | {label(d)} | {d['kind']} | {t(d['quarter_s'])} | {speed(d['quarter_kmh'])} | {t(d['t_100kmh'])} |")
    L.append("")

    L.append("## Top speed board\n")
    L.append("| # | Vehicle | Type | Top speed | Held in |")
    L.append("|---|---|---|---|---|")
    for k, d in enumerate(sorted(results, key=lambda d: -d["top_kmh"]), 1):
        L.append(f"| {k} | {label(d)} | {d['kind']} | {speed(d['top_kmh'])} | {top_how(d)} |")
    L.append("")

    # per vehicle
    L.append("## Vehicles\n")
    for vid, ds in by_id.items():
        stock = next((d for d in ds if d["variant"] == "stock"), ds[0])
        L.append(f"### {stock['name']}\n")
        L.append(f"{stock['summary']}  ")
        L.append(f"`./engine_sim --engine-type {vid}`" +
                 "".join(f", `--variant {d['variant']}`" if d['variant'] != 'tuned' else ", `--tuned yes`"
                         for d in ds if d is not stock) + "\n")
        others = [d for d in ds if d is not stock]
        head = "| | Stock |" + "".join(f" {d['variant'].capitalize()} | Change |" for d in others)
        L.append(head)
        L.append("|---|---|" + "---|---|" * len(others))
        for name, fmt, key, unit, dfmt in ROWS:
            row = f"| {name} | {fmt(stock)} |"
            for d in others:
                row += f" {fmt(d)} | {delta(stock, d, key, unit, dfmt) if key else ''} |"
            L.append(row)
        L.append("")

    with open(a.out, "w") as f:
        f.write("\n".join(L))
    print(f"wrote {a.out}", file=sys.stderr)


if __name__ == "__main__":
    main()
