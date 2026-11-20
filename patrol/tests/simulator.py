"""TCP floor simulator. Run make sim first. --bind 0.0.0.0 for a phone on your LAN."""
import argparse
import ctypes as ct
import selectors
import socket
import time
from pathlib import Path

def serve(bind, port):
    sim = ct.CDLL(str(Path(__file__).resolve().parents[1] / "build/sim_car.so"))
    sim.sim_line.argtypes = [ct.c_char_p, ct.c_uint32]
    sim.sim_line.restype = ct.c_char_p
    sim.sim_reset()
    with socket.create_server((bind, port)) as server, selectors.DefaultSelector() as sel:
        sel.register(server, selectors.EVENT_READ)
        peer, buf = None, bytearray()
        start = time.monotonic()
        print(f"巡逻模拟器 {bind}:{port}", flush=True)
        while True:
            events = sel.select(.01)
            now = int((time.monotonic() - start) * 1000)
            sim.sim_tick(now)
            for key, _ in events:
                if key.fileobj is server:
                    client, _ = server.accept()
                    if peer:
                        client.close()
                    else:
                        peer = client
                        peer.settimeout(.05)
                        sel.register(peer, selectors.EVENT_READ)
                    continue
                try:
                    data = peer.recv(256)
                    if not data:
                        raise EOFError
                    buf.extend(data)
                    while b"\n" in buf:
                        line, _, rest = buf.partition(b"\n")
                        buf[:] = rest
                        response = sim.sim_line(bytes(line), now)
                        if not response or b"\x00" in line:
                            raise ValueError
                        peer.sendall(response)
                    if len(buf) > 39:
                        raise ValueError
                except (OSError, EOFError, ValueError):
                    sel.unregister(peer)
                    peer.close()
                    peer = None
                    buf.clear()
                    sim.sim_line(b"0 S", now)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7070)
    args = parser.parse_args()
    serve(args.bind, args.port)
