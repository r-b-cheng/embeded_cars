"""Run the station AT transcript and delayed upload ACK on emulated UART1."""
import re
import subprocess
from pathlib import Path
from tempfile import TemporaryDirectory

root = Path(__file__).resolve().parents[2]
elf = root / "build/renode.elf"
symbols = {name:int(addr,16) for addr, kind, name in
           (line.split() for line in subprocess.check_output(
               ["arm-none-eabi-nm","--defined-only",elf], text=True).splitlines())}

with TemporaryDirectory() as temp:
    folder = Path(temp)
    uart = folder/"uart.txt"
    commands = ["mach create",f"machine LoadPlatformDescription @{root}/tests/renode/patrol.repl",
                "sysbus WriteDoubleWord 0x400fe000 0x10050000",f"sysbus LoadELF @{elf}",
                f"sysbus.uart1 CreateFileBackend @{uart} true"]
    def run(seconds):
        commands.append(f'emulation RunFor "{seconds}"')
    def receive(data):
        # Fit the peripheral RX FIFO; allow the actual ISR to service each chunk.
        framed=data.encode("ascii")
        for pos in range(0,len(framed),8):
            commands.extend(f"sysbus.uart1 WriteChar {b}" for b in framed[pos:pos+8])
            run("0.002")
    def ipd(seq,action):
        payload=f"{seq} W {action}\n"
        receive(f"+IPD,4,{len(payload)}:"+payload)

    run("1.6")
    receive("ready\r\n")
    for _ in range(4): receive("OK\r\n")
    receive('+CIPSTA:gateway:"192.168.43.1"\r\nOK\r\n')
    for _ in range(4): receive("OK\r\n")
    run("3.0")
    receive("4,CONNECT\r\nOK\r\n")
    ipd(1,"S"); receive("> ")
    ipd(2,"F")  # arrives before preceding SEND OK
    receive("SEND OK\r\n"); receive("> "); run("0.16")
    for seq in range(3,19):
        receive("SEND OK\r\n"); ipd(seq,"F"); receive("> "); run("0.055")
    receive("SEND OK\r\n"); ipd(19,"F")
    commands.append(f"sysbus WriteDoubleWord {symbols['block_tx']} 1")
    receive("> ")  # TX FIFO remains unavailable for the entire lease
    run("0.92")
    commands.append(f"sysbus WriteDoubleWord {symbols['block_tx']} 0")
    run("0.58")  # allow the queued reply out, withhold SEND OK

    names = ["lease_delay","max_interval","moving_ticks","stopped_outputs","fifo_refusals"]
    commands.extend(f"sysbus ReadDoubleWord {symbols[n]}" for n in names)
    commands.append("quit")
    script = folder/"test.resc"; script.write_text("\n".join(commands)+"\n")
    proc=subprocess.run(["renode","--disable-gui","--plain","--console",script],
                        capture_output=True,text=True,timeout=55)
    output=proc.stdout+proc.stderr
    wire=uart.read_text() if uart.exists() else ""
    values=[int(v,16) for v in re.findall(r"^0x([0-9A-Fa-f]+)\s*$",output,re.M)]
    try:
        assert proc.returncode==0 and len(values)==5
        delay,interval,ticks,stopped,refusals=values
        assert 'AT+CWJAP="example","test-only"' in wire and 'AT+CIPSTART=4,"TCP","192.168.43.1",7071' in wire
        frames=[line.split() for line in wire.splitlines() if line.startswith("T ")]
        assert [int(f[1]) for f in frames]==list(range(1,20)) and all(len(f)==64 for f in frames)
        assert frames[-2][2]=="1" and int(frames[-1][22])>0 and int(frames[-1][37])>0
        assert [int(f[33]) for f in frames]==list(range(1,20))
        assert int(frames[0][63])>=50 and all(int(f[58]) for f in frames)
        assert wire.count("AT+CIPSEND=4,")==len(frames)
        for advertised,f in zip(re.findall(r"AT\+CIPSEND=4,(\d+)",wire),frames):
            assert int(advertised)==len(" ".join(f)+"\n")
        assert 650<delay<=670 and interval<=15 and ticks>5 and stopped==1 and 0<refusals<=1000
    except (ValueError,AssertionError):
        print(output[-4000:],wire,values); raise
    print(f"Renode Cortex-M4 / UART1: 19 full telemetry + journal replies, control max {interval} ms, "
          f"lease stop {delay} ms, wheels zero; {ticks} moving ticks, {refusals} FIFO refusals.")
