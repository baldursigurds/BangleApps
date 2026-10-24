#!/usr/bin/env python3
"""Build puckcode.js - the module with the code footrun uploads to the Puck -
from puck.js and puck.c.

puck.c is the compiled part, which on its own the Espruino Web IDE would
compile online with E.compiledC - a watch can't. So this does what
Espruino's compiler (EspruinoCompiler, github.com/gfwilliams/EspruinoCompiler)
does, with the same gcc flags and linker script: compile it for a Cortex-M4,
and replace PUCK_C in puck.js with E.nativeCall()s for its functions. The C
uses no Espruino functions, so the binary works on any firmware version.

puck.js is then minified (comments and spaces removed, with the Esprima and
escodegen that come with EspruinoTools, as the Web IDE does), and both are
written to puckcode.js, which the App Loader installs as the module
"footrun.puck":

    exports.js      footrun.js on the Puck: the code (a string)
    exports.jsCrc   its CRC32
    exports.bin     footrun.bin on the Puck: the compiled C (base64)
    exports.binCrc  its CRC32

footrun writes them to the Puck's Storage (if they've changed) and runs
eval(require("Storage").read("footrun.js")) - code run from Storage keeps
its functions in flash, and E.nativeCall runs the binary straight from
flash, so it all fits in the Puck's ~33kB of RAM (sent as one line it
doesn't: the line, its parsed functions and the binary all in RAM).

    python3 build.py        # after changing puck.js or puck.c

Needs arm-none-eabi-gcc, node, and EspruinoTools (npm install -g espruino).
"""

import base64
import json
import os
import re
import struct
import subprocess
import tempfile
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
PUCK_JS = os.path.join(HERE, "puck.js")
PUCK_C = os.path.join(HERE, "puck.c")
MODULE = os.path.join(HERE, "puckcode.js")
BIN_NAME = "footrun.bin"  # the compiled C's name in the Puck's Storage

# From EspruinoCompiler: src/compile.js (cortexm4) and inc/linker.ld - except
# libc.a/libm.a/libgcc.a in /DISCARD/, which make ld look for those libraries
# (that may not be installed); with -nostdlib nothing from them is linked anyway
CFLAGS = ("-mlittle-endian -mthumb -mthumb-interwork -mcpu=cortex-m4 -mfloat-abi=softfp -mfpu=fpv4-sp-d16 "
          "-nostdinc -nostdlib -fno-common -fno-exceptions -fdata-sections -ffunction-sections "
          "-flto -fno-fat-lto-objects -Wl,--allow-multiple-definition -fpic -fpie -fpermissive "
          "-fpreprocessed -std=c++17 -Os").split()
LINKER_LD = """ENTRY(entryPoint)
MEMORY { RAM (xrw) : ORIGIN = 0, LENGTH = 100K }
SECTIONS {
  .entrypoint : { . = ALIGN(4); *(.entryPoint) }
  .text : {
    . = ALIGN(4); *(.text) *(.text*) *(.rodata) *(.rodata*)
    . = ALIGN(4); *(.data) *(.data*)
    . = ALIGN(4); *(.bss) *(.bss*) *(COMMON)
    . = ALIGN(4);
  } >RAM
  /DISCARD/ : { *(.ARM.*) *(.comment) }
}
"""


def compile_c(code):
    """(JS for what E.compiledC(code) returns, machine code, functions), as
    EspruinoCompiler makes them - but with the machine code read from the
    Puck's Storage rather than included in the JS."""
    entry_points, entry_specs = [], []
    for line in code.strip().split("\n"):
        line = line.strip()
        if not line.startswith("//"):
            break
        m = re.match(r"//\s*(\w+)\s+(\w+)\(([^)]*)\)", line)
        if not m:
            raise SystemExit(f"puck.c: {line!r} isn't a valid function signature (the first lines must all be)")
        entry_points.append(m.group(2))
        entry_specs.append(f"{m.group(1)}({m.group(3)})")
    if not entry_points:
        raise SystemExit("puck.c must start with lines like '// void process(int, int, int)'")
    code += ('\nextern "C" { void *entryPoint[]  __attribute__ ((section (".entrypoint"))) = {(void*)'
             + ",(void*)".join(entry_points) + "};}")
    with tempfile.TemporaryDirectory() as d:
        open(os.path.join(d, "out.cpp"), "w").write(code)
        open(os.path.join(d, "linker.ld"), "w").write(LINKER_LD)
        subprocess.run(["arm-none-eabi-gcc", *CFLAGS, "-T" + os.path.join(d, "linker.ld"),
                        os.path.join(d, "out.cpp"), "-o", os.path.join(d, "out.elf")], check=True)
        undefined = subprocess.run(["arm-none-eabi-nm", "-u", os.path.join(d, "out.elf")],
                                   capture_output=True, text=True, check=True).stdout.strip()
        if undefined:
            raise SystemExit("puck.c needs functions there's no library for:\n" + undefined)
        subprocess.run(["arm-none-eabi-objcopy", "-O", "binary", os.path.join(d, "out.elf"),
                        os.path.join(d, "out.bin")], check=True)
        binary = open(os.path.join(d, "out.bin"), "rb").read()
    # The binary starts with the table of function addresses
    offset = len(entry_points) * 4
    src = f'(function(){{var bin=require("Storage").read("{BIN_NAME}");return{{'
    for i, (name, spec) in enumerate(zip(entry_points, entry_specs)):
        addr = struct.unpack_from("<i", binary, i * 4)[0] - offset
        if not addr & 1:
            raise SystemExit(f"{name}'s address {addr} isn't a Thumb address - the build went wrong")
        src += f"{name}:E.nativeCall({addr},{json.dumps(spec)},bin),"
    return src + "};})()", binary[offset:], dict(zip(entry_points, entry_specs))


def minify(js):
    """Comments and spaces removed, as the Web IDE's 'Esprima (offline)' option."""
    root = subprocess.run(["npm", "root", "-g"], capture_output=True, text=True).stdout.strip()
    libs = os.path.join(root, "espruino", "libs", "esprima")
    if not os.path.exists(libs):
        raise SystemExit("Needs EspruinoTools for Esprima/escodegen: npm install -g espruino")
    script = f"""
const esprima = require({json.dumps(os.path.join(libs, "esprima.js"))});
// a browser bundle that sets the global escodegen
require("vm").runInThisContext(require("fs").readFileSync({json.dumps(os.path.join(libs, "escodegen.js"))}, "utf8"));
let code = require("fs").readFileSync(0, "utf8");
const ast = esprima.parse(code, {{ raw: true }});
process.stdout.write(escodegen.generate(ast, {{ format: {{ compact: true, semicolons: true, quotes: "auto",
  indent: {{ style: "" }}, hexadecimal: true, renumber: true, escapeless: false, parentheses: false }} }}));
"""
    return subprocess.run(["node", "-e", script], input=js, capture_output=True, text=True, check=True).stdout


def main():
    js = open(PUCK_JS).read()
    if len(re.findall(r"\bPUCK_C\b", js)) != 1:
        raise SystemExit("puck.js must use PUCK_C exactly once, where the compiled puck.c goes")
    compiled, binary, functions = compile_c(open(PUCK_C).read())
    code = re.sub(r"\bPUCK_C\b", lambda _m: compiled, js)
    small = minify(code)
    subprocess.run(["node", "--check", "-"], input=small, text=True, check=True)
    if any(ord(ch) > 126 for ch in small):
        raise SystemExit("The minified code has non-ASCII characters")
    module = (
        "// Made by build.py from puck.js and puck.c - don't edit. The App Loader installs\n"
        "// it as the module \"footrun.puck\": the files footrun puts on the Puck.\n"
        "exports.js = " + json.dumps(small) + "; // footrun.js\n"
        "exports.jsCrc = " + str(zlib.crc32(small.encode())) + ";\n"
        "exports.bin = " + json.dumps(base64.b64encode(binary).decode()) + "; // footrun.bin, base64\n"
        "exports.binCrc = " + str(zlib.crc32(binary)) + ";\n")
    open(MODULE, "w").write(module)
    print(f"puck.c: {len(binary)} bytes of ARM code, functions {functions}")
    print(f"puck.js: {len(js)} bytes -> {len(small)} bytes minified")
    print(f"wrote {os.path.relpath(MODULE)} ({len(module)} bytes)")


if __name__ == "__main__":
    main()
