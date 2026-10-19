"""Builds the 3D PlayTest app: sdk/build/apps/3D-PlayTest/3D-PlayTest.bin.

1. picasso (devkitPro's PICA200 assembler) assembles playtest.v.pica, and
   tools/shbin2c.py turns it into gen/playtest_shbin.h.
2. gen/assets.s brings assets/player.glb and assets/test_room.glb into the
   app with .incbin.
3. sdk/aurcc.py builds this folder together with the OS's PICA200 command
   list builder (src/os/P3d9.c) and 3D screen control (src/os/Stereo9.c).

Run from anywhere: py sdk/examples/3D-PlayTest/build.py [-v]
"""
import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
NAME = "3D-PlayTest"
OUT = REPO / "sdk" / "build" / "apps" / NAME
GEN = OUT / "gen"
ASSETS = {"playtest_player_glb": HERE / "assets" / "player.glb",
          "playtest_room_glb": HERE / "assets" / "test_room.glb"}

verbose = "-v" in sys.argv[1:]


def run(cmd, **kw):
    cmd = [str(c) for c in cmd]
    if verbose:
        print("  $ " + " ".join(cmd))
    r = subprocess.run(cmd, **kw)
    if r.returncode:
        sys.exit(r.returncode)
    return r


def find_picasso():
    found = shutil.which("picasso")
    if found:
        return found
    roots = []
    dkp = os.environ.get("DEVKITPRO", "")
    if dkp.startswith("/opt/devkitpro"):  # an MSYS path
        dkp = "C:/devkitPro" + dkp[len("/opt/devkitpro"):]
    if dkp:
        roots.append(Path(dkp))
    roots.append(Path("C:/devkitPro"))
    for root in roots:
        for exe in ("picasso.exe", "picasso"):
            p = root / "tools" / "bin" / exe
            if p.is_file():
                return str(p)
    sys.exit("picasso was not found: install devkitPro's 3DS tools "
             "(3ds-dev), which put it in devkitPro/tools/bin")


def shader():
    shbin = GEN / "playtest.shbin"
    run([find_picasso(), "-o", shbin, HERE / "playtest.v.pica"])
    r = run([sys.executable, REPO / "tools" / "shbin2c.py", shbin,
             "playtest_shader"], capture_output=True)
    (GEN / "playtest_shbin.h").write_bytes(r.stdout.replace(b"\r\n", b"\n"))


def assets():
    lines = ['.section .rodata.playtest_assets, "a"']
    for sym, path in ASSETS.items():
        if not path.is_file():
            sys.exit(f"missing {path}")
        lines += [".balign 4", f".global {sym}", f".global {sym}_end",
                  f"{sym}:", f'.incbin "{path.as_posix()}"', f"{sym}_end:"]
    (GEN / "assets.s").write_bytes(("\n".join(lines) + "\n").encode("ascii"))


def main():
    GEN.mkdir(parents=True, exist_ok=True)
    shader()
    assets()
    run([sys.executable, REPO / "sdk" / "aurcc.py", "build", HERE,
         REPO / "src" / "os" / "P3d9.c", REPO / "src" / "os" / "Stereo9.c",
         GEN / "assets.s",
         "-I", REPO / "include", "-I", GEN, "-I", HERE / "lib" / "cgltf",
         "--icon", HERE / f"{NAME}.icon",
         "-o", OUT / f"{NAME}.bin", *(["-v"] if verbose else [])])


if __name__ == "__main__":
    main()
