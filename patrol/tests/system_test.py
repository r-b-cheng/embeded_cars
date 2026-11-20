"""Exercise production C motion logic and Android pilot together, at simulated time."""
import ctypes as ct
import math
import subprocess
from pathlib import Path

root = Path(__file__).resolve().parents[1]
sim = ct.CDLL(str(root / "build/sim_car.so"))
sim.sim_line.argtypes = [ct.c_char_p, ct.c_uint32]
sim.sim_line.restype = ct.c_char_p
for mission in ("route", "explore", "detour"):
    sim.sim_reset()
    if mission == "detour":
        sim.sim_set_course()
    app = subprocess.Popen(["java", "-ea", "-cp", str(root/"build/java"),
                            "org.embeddedcars.patrol.NavigationTest", mission],
                           stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    cmd = b"S"
    farthest = 0
    try:
        for frame in range(1000 if mission == "detour" else 240):
            for j in range(10):
                now = frame*100 + 10*j
                sim.sim_tick(now)
            app.stdin.write(sim.sim_line(b"42 "+cmd, now))
            app.stdin.flush()
            action, x, state = app.stdout.readline().decode().strip().split("|", 2)
            cmd = action.encode()
            farthest = max(farthest, int(x))
            if state == "路线已完成":
                break
        if mission == "route":
            assert cmd == b"S" and state == "路线已完成" and 700 <= int(x) <= 900
        elif mission == "detour":
            assert cmd == b"S" and state == "路线已完成" and sim.sim_contacts() == 0, (farthest, state, sim.sim_contacts())
        else:
            assert farthest >= 600, (mission, farthest, state)
        gx, gy = sim.sim_true_x(), sim.sim_true_y()
        if mission != "explore":
            target = 800 if mission == "route" else 1600
            assert math.hypot(gx-target, gy) < 250 and sim.sim_contacts() == 0, (gx, gy)
        print("true (mm)", gx, gy, mission, "max x (estimated mm)", farthest, state)
    finally:
        app.stdin.close()
        assert app.wait(timeout=5)==0
