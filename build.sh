#!/bin/sh
# Build px68k_libretro.wasm (drop-in for the WebX68k Emscripten glue) using zig cc (pip install ziglang).
set -e
cd "$(dirname "$0")"
ZIG="${ZIG:-python3 -m ziglang}"
OUT="${1:-build/px68k_libretro.wasm}"
mkdir -p "$(dirname "$OUT")"
EXPORTS=$(grep -ho 'EXPORT [a-z_ *0-9A-Z]* \**\([a-z_0-9]*\)(' src/core.c | sed 's/.*[ *]\([a-z_0-9]*\)($/\1/' | sort -u)
EXPORTS="$EXPORTS __wasm_call_ctors malloc free get_mouse_scc_x get_mouse_scc_y get_mouse_scc_stat get_mouse_enabled webx68k_dummy"
LDEXP=""
for e in $EXPORTS; do [ "$e" = "webx68k_dummy" ] || LDEXP="$LDEXP -Wl,--export=$e"; done
$ZIG cc -target wasm32-freestanding -O2 -fno-builtin -ffreestanding -nostdlib -fno-stack-protector \
  -Wall -Wno-unused-function -Wno-misleading-indentation \
  -Wl,--no-entry -Wl,--export-table -Wl,--growable-table -Wl,--export-memory -Wl,--initial-memory=33554432 \
  -Wl,--max-memory=1073741824 -Wl,-z,stack-size=1048576 -Wl,--gc-sections $LDEXP \
  -o "$OUT" src/cpu.c src/machine.c src/periph.c src/video.c src/sound.c src/dmac.c src/fdc.c src/sasi.c src/host.c src/core.c
ls -la "$OUT"
