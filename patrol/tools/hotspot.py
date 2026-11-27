"""Create the private header for 'make personal' without echoing the password."""
import getpass
import json
import os
from pathlib import Path

def argument(value, low, high):
    if not low <= len(value.encode()) <= high or any(ord(c) < 32 for c in value):
        raise ValueError("热点名称或口令长度、字符无效")
    for c in ('\\', '"', ','):
        value = value.replace(c, '\\' + c)
    return '"' + value + '"'

if __name__ == "__main__":
    ssid = argument(input("热点名称："), 1, 32)
    password = argument(getpass.getpass("WPA2 口令："), 8, 63)
    folder = Path(__file__).resolve().parents[1] / "build/personal"
    os.umask(0o077)
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "hotspot.h").write_text(
        "#define HOTSPOT_ARGUMENTS " + json.dumps(ssid + "," + password, ensure_ascii=False) + "\n")
