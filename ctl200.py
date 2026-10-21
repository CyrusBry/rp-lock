"""
ctl200.py -- minimal serial helper for the Koheron CTL200 (Build 2B, PLANEX on CTL200-2-B-200)

Usage (Anaconda PowerShell, pyrpl-env):
    pip install pyserial
    python -i ctl200.py COM5          # COMx from Device Manager; on the Mac: /dev/tty.usbmodem*

Then in the Python shell:
    cmd("status")                     # any raw command, reply printed
    setup()                           # limits + thermistor set point, laser stays OFF
    tec_on()                          # waits until rtact is within 10 ohm of rtset
    laser_up(100)                     # steps 10/20/50/80/100 mA, prints vlaser + iphd at each
    laser_off()

All commands are the CTL200 serial set (115200 8N1, CRLF-terminated).
Set points are in OHMS (rtset 11460 = 11.460 kOhm = 22 C on the PLANEX test report).
"""
import sys, time
import serial

# --- PLANEX SN 120025 numbers (RIO test report) -------------------------------
RT_SET_OHM   = 11460.0     # 22.0 C set point
RT_MIN_OHM   = 8000.0      # laser disables outside this window (~30 C)
RT_MAX_OHM   = 16000.0     # (~15 C)
I_MAX_MA     = 125.0       # software limit; RIO absolute max 150, bias 120, we run 100
I_STEPS_MA   = [10, 20, 50, 80, 100]
V_LASER_MAX  = 2.0         # abort if forward voltage exceeds this (bad contact / wrong pins)

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM5"
ser  = serial.Serial(PORT, 115200, timeout=0.3)

def cmd(s, quiet=False):
    """Send one command, return the reply text (echo + value + prompt stripped)."""
    ser.reset_input_buffer()
    ser.write((s + "\r\n").encode())
    time.sleep(0.15)
    raw = ser.read(ser.in_waiting or 1).decode(errors="replace")
    lines = [l.strip() for l in raw.replace(">>", "").splitlines() if l.strip()]
    if lines and lines[0] == s:
        lines = lines[1:]
    reply = " ".join(lines)
    if not quiet:
        print(f"{s:>12} -> {reply}")
    return reply

def num(s):
    try:
        return float(cmd(s, quiet=True).split()[0])
    except Exception:
        return float("nan")

def setup():
    """Limits and set points. Laser stays off. Save with lason 0 so a reboot never lights the laser."""
    cmd("version")
    cmd("lason 0")
    cmd(f"ilmax {I_MAX_MA}")
    cmd("ilaser 0")
    cmd(f"rtmin {RT_MIN_OHM}")
    cmd(f"rtmax {RT_MAX_OHM}")
    cmd("vtmin -2.0"); cmd("vtmax 2.0")
    cmd("tprot 1")
    cmd(f"rtset {RT_SET_OHM}")
    cmd("save")
    print("saved: laser OFF, ilmax %.0f mA, rtset %.0f ohm" % (I_MAX_MA, RT_SET_OHM))

def tec_on(tol_ohm=10.0, timeout_s=120):
    cmd("tecon 1")
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        r, i, v = num("rtact"), num("itec"), num("vtec")
        print(f"  rtact {r:8.1f} ohm  itec {i:+.3f} A  vtec {v:+.3f} V")
        if abs(r - RT_SET_OHM) < tol_ohm:
            print("TEC settled."); return True
        if abs(i) > 1.0:
            print("TEC current > 1 A and not converging -- check TEC polarity/thermistor. Stopping TEC.")
            cmd("tecon 0"); return False
        time.sleep(2)
    print("TEC did not settle in time."); return False

def laser_up(i_final=100.0, hold_s=10):
    if abs(num("rtact") - RT_SET_OHM) > 20:
        print("Thermistor not at set point -- run tec_on() first."); return
    cmd("ilaser 0"); cmd("lason 1")
    for i in [s for s in I_STEPS_MA if s <= i_final] + ([i_final] if i_final not in I_STEPS_MA else []):
        cmd(f"ilaser {i}")
        time.sleep(1.0)
        vl, ipd = num("vlaser"), num("iphd")
        print(f"  I = {i:6.1f} mA   vlaser = {vl:.3f} V   iphd = {ipd:.3f} mA")
        if vl > V_LASER_MAX or vl != vl:
            print("vlaser out of range -- laser OFF."); cmd("lason 0"); return
        time.sleep(hold_s)
    cmd("status")

def laser_off():
    cmd("ilaser 0"); cmd("lason 0")

print(f"CTL200 on {PORT}.  Commands: cmd('status'), setup(), tec_on(), laser_up(100), laser_off()")
cmd("version")
