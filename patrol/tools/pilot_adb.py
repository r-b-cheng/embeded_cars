"""Read observations and issue one bounded decision through an authorized phone."""
import argparse
import base64
import json
import re
import shlex
import subprocess

def bridge(method, decision=None, serial=None):
    cmd = ["content","call","--uri","content://org.embeddedcars.patrol.pilot","--method",method]
    if decision is not None:
        arg = base64.urlsafe_b64encode(json.dumps(decision,ensure_ascii=False).encode()).decode()
        cmd += ["--arg",arg]
    adb = ["adb","-s",serial] if serial else ["adb","-d"]
    result = subprocess.check_output(adb+["shell",shlex.join(cmd)],text=True,timeout=7)
    match = re.search(r"payload=([A-Za-z0-9_=-]+)",result)
    if not match:
        raise RuntimeError(result.strip())
    return json.loads(base64.urlsafe_b64decode(match[1]))

if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("method",choices=["snapshot","decide","stop"])
    p.add_argument("--serial")
    p.add_argument("--ticket")
    p.add_argument("--action",choices=["forward","turn","wait","stop"])
    p.add_argument("--value",type=int,default=0)
    p.add_argument("--reason",default="电脑决策")
    args=p.parse_args()
    decision=None
    if args.method == "decide":
        if not args.ticket or not args.action: p.error("decide requires --ticket and --action")
        decision=dict(ticket=args.ticket,action=args.action,value=args.value,reason=args.reason)
    print(json.dumps(bridge(args.method,decision,args.serial),indent=2,ensure_ascii=False))
