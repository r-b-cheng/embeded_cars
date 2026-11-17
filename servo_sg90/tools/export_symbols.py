"""Export the volatile control mailbox addresses together with its image hash."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
for name in ("nm", "elf", "binary", "output"):
    parser.add_argument("--" + name, required=True)
args = parser.parse_args()
symbols = {}
for line in subprocess.check_output([args.nm, "-n", args.elf], text=True).splitlines():
    fields = line.split()
    if len(fields) == 3 and fields[2].startswith("control_"):
        symbols[fields[2]] = int(fields[0], 16)
expected = {"control_target_us", "control_enable", "control_active", "control_applied_us", "control_millis"}
if set(symbols) != expected or not all(0x20000000 <= address < 0x20008000 for address in symbols.values()):
    raise SystemExit("Unexpected control mailbox symbols")
data = {"image_sha256": hashlib.sha256(Path(args.binary).read_bytes()).hexdigest(), "symbols": symbols}
Path(args.output).write_text(json.dumps(data, indent=2) + "\n")
