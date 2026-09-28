#!/usr/bin/env python3
"""
Build 1A: PLANEX self-heterodyne delay-line lock closed with a Red Pitaya
(STEMlab 125-14 PRO Gen 2) running PyRPL.

Signal chain (UCLA Build.pdf, Fig. 2):
    OUT2  -> low-pass -> ~20 dB pad -> ZX60-100VH+ -> AOM      (40 MHz drive)
    IN1   <- PDA05CF2 (SM1FCA, VOA) -> BLP-100+ -> DC block -> tee + 50 ohm
    OUT1  -> 4.7 kohm series -> tee + 50 ohm terminator -> CLD1015 Mod In   (100:1, as built 2026-09-23)
             Direct drive was tried 2026-09-22 and is impossible: the required P (5e-6) is below the
             PID register floor (2.44e-4). 47 kohm (1000:1) locks but rails at +/-5 MHz. -> DIVIDER below.

The IQ module both GENERATES the 40 MHz drive (its own output on OUT2) and
DEMODULATES the beat on IN1 against the same NCO, so drive phase noise is
common-mode.  The demodulated quadrature (~ cos(phi)) is the error signal,
which pid0 closes on OUT1.

Usage:
    python build1a_pyrpl.py sweep       # open loop: show the fringe, report V_max / V_min
    python build1a_pyrpl.py lock        # close the loop step by step
    python build1a_pyrpl.py unlock      # open the loop safely (output to 0 V)
    python build1a_pyrpl.py status      # print module settings + error/servo RMS
    python build1a_pyrpl.py calib       # measure actuator gain [MHz/V at OUT1] by fringe counting
    python build1a_pyrpl.py shell       # ONE persistent connection: status()/lock()/seti()/unlock() at a prompt (preferred for tuning)

Edit HOSTNAME and the LEVELS block below before the first run.
"""

import sys
import time
import numpy as np

# ---------------------------------------------------------------- settings
HOSTNAME = "192.168.10.2"         # static IP on the direct link to the lab PC (PC port = 192.168.10.1)
CONFIG   = "ucla_build1a"         # PyRPL config file name (created on first run)

F_AOM        = 40.000e6           # [Hz]  AOM drive = IQ demodulation frequency
DRIVE_AMPL   = 1.00               # [V]   bench 2026-09-16: 1.0 -> -13.4 dBm at the 22 dB pad output (0.85 gave -14.8 dBm)
DRIVE_MAX    = 1.00               # [V]   hard cap (PyRPL clamps at 1.0 anyway); 22 dB pad keeps the ZX60 input < +15 dBm

IQ_BANDWIDTH = 300e3              # [Hz]  demodulation low-pass (one pole). 200-500 kHz is fine.
IQ_ACBW      = 2e6                # [Hz]  input high-pass before demod (removes residual DC / LF)
IQ_QFACTOR   = 2.7                # [-]   bench 2026-09-22 (PDA05CF2 via VOA): fringe +/-0.060 at qf = 1 -> +/-0.178 at 2.7 (qf = 20 clipped at +/-1)

# ---- actuator scaling ---------------------------------------------------
# Measured 2026-09-16: PLANEX ~30 MHz/mA through the CLD1015 (150 mA/V) = ~5 GHz/V at the Mod In.
# DIVIDER = 1.0   : OUT1 straight into the Mod In (through a tee + 50 ohm terminator)
# DIVIDER = 0.01  : 4.7 kohm series + 50 ohm shunt at the Mod In (100:1)
# DIVIDER = 0.001 : 47 kohm series + 50 ohm shunt at the Mod In (1000:1, the first-fringe setup; +/-5 MHz range)
DIVIDER      = 0.01               # as built 2026-09-23: 4.7 kohm series, 50 ohm shunt -> 1:95, +/-53 MHz at +/-1 V, 8 LSB per fringe
K_ACT_MODIN  = 5.0e9              # [Hz/V] at the Mod In (measured; refine with 'calib')
K_ACT_OUT1   = K_ACT_MODIN * DIVIDER   # [Hz/V] referred to OUT1: 5 GHz/V direct, 50 MHz/V with 100:1

FRINGE_HZ    = 102.0e3            # [Hz]  1/tau_d for the 2 km spool (tau_d = 9.8 us)
I_MAX_HZ     = 40.0               # [Hz]  hard cap for seti(): I = 25 Hz already puts the crossover at ~12 kHz through the
                                  #       100:1 plant gain (~500). I = 100 Hz -> 50 kHz = the delay line's -180 deg point.
LAST_FRINGE  = {'ampl': None}     # set by sweep(); status() reports suppression against ampl/sqrt(2)

OUT1_LIMIT   = 0.1 if DIVIDER >= 0.5 else 1.0   # [V] clamp: +/-0.1 V direct = +/-15 mA (laser stays lit); +/-1 V with the divider

SWEEP_FREQ   = 10.0               # [Hz]  open-loop sweep on OUT1 (through the divider = a few fringes)
SWEEP_AMPL   = 3.0e6 / K_ACT_OUT1 # [V]   ~3 MHz peak = ~30 fringes each way (0.6 mV direct = 2.5 LSB!, 60 mV with 100:1)

# Loop-shaping start points, derived from the measured actuator (above).
# A flat proportional loop through 9.8 us of delay cannot exceed |L| ~ 2, so P
# only supplies ~0.25 of loop gain and the integrator does the holding.
# Loop gain L_P = P * K_d * 2*pi*tau_d * K_ACT_OUT1 ~ P * 0.16 * 6.2e-5 * K_ACT_OUT1  (= 5e4*P direct, 500*P with 100:1)
P_START      = 0.25 / (0.16 * 6.16e-5 * K_ACT_OUT1)   # L_P ~ 0.25  (5e-6 direct, 5e-4 with 100:1)
P_STEPS      = []                 # empty = close at P_START and go straight to the integrator
I_UNITY_HZ   = 12.5e3 / (0.16 * 6.16e-5 * K_ACT_OUT1)  # [Hz] crossover ~12 kHz  (0.25 Hz direct, 25 Hz with 100:1). Raise to broadening, back off 30 %

# ---------------------------------------------------------------- helpers
def connect():
    from pyrpl import Pyrpl
    p = Pyrpl(config=CONFIG, hostname=HOSTNAME, gui=False)
    return p, p.rp


def setup_drive_and_demod(rp):
    """IQ module: 40 MHz LO out on OUT2, demodulate IN1, quadrature -> signal bus."""
    iq = rp.iq0
    amp = min(DRIVE_AMPL, DRIVE_MAX)
    iq.setup(frequency=F_AOM,
             bandwidth=[IQ_BANDWIDTH],      # demod LPF corner(s)
             gain=0.0,                      # no re-modulated feedback on the output
             phase=0,
             acbandwidth=IQ_ACBW,           # input HPF
             amplitude=amp,                 # LO amplitude on the direct output  [V]
             input='in1',
             output_direct='out2',          # the AOM drive
             output_signal='quadrature',    # what pid0 sees
             quadrature_factor=IQ_QFACTOR)
    return iq


def safe_outputs(rp):
    """Make OUT1 safe BEFORE anything else touches the actuator: servo off, sweep off,
    integrator zero, clamp at +/-OUT1_LIMIT, gains at the current config."""
    pid = rp.pid0
    pid.output_direct = 'off'
    rp.asg1.output_direct = 'off'
    pid.ival = 0.0
    pid.p = 0.0
    pid.i = 0.0
    pid.max_voltage = OUT1_LIMIT
    pid.min_voltage = -OUT1_LIMIT
    return pid


def setup_pid(rp, p=P_START, i=0.0):
    pid = safe_outputs(rp)                  # clamp first, always
    pid.input = 'iq0'
    pid.setpoint = 0.0                      # quadrature ~ cos(phi): lock at a zero crossing
    pid.inputfilter = [0, 0, 0, 0]
    pid.p = p
    pid.i = i
    pid.ival = 0.0                          # reset integrator
    pid.output_direct = 'off'               # closed later
    return pid


def sweep_on(rp):
    asg = rp.asg1
    asg.setup(waveform='ramp', frequency=SWEEP_FREQ, amplitude=SWEEP_AMPL,   # 'ramp' = triangle in PyRPL
              offset=0.0, output_direct='out1', trigger_source='immediately')
    return asg


def sweep_off(rp):
    rp.asg1.output_direct = 'off'


def scope_curve(rp, ch1='iq0', ch2='pid0', duration=0.2):
    """Blocking single acquisition; returns (ch1, ch2) as numpy arrays."""
    sc = rp.scope
    sc.input1 = ch1
    sc.input2 = ch2
    sc.duration = duration
    sc.trigger_source = 'immediately'
    sc.average = True     # boxcar-average each decimation block = anti-alias filter BEFORE the 5 kHz
                          # brick wall. With False (through 2026-09-23) the IQ output noise up to 300 kHz
                          # folded into baseband and "err RMS < 5 kHz" sat on a ~0.06 V floor, locked or not.
    sc.rolling_mode = False
    data = np.asarray(sc.single())          # PyRPL >= 0.9.4: single() blocks and returns the trace(s)
    if data.ndim == 2:                       # shape (2, N): ch1, ch2
        return data[0], data[1]
    return data, np.zeros_like(data)         # single-channel return on some builds



def lf_rms(x, duration, fc=5e3):
    """RMS of x after a brick-wall low-pass at fc [Hz] (FFT). duration = trace length [s].
    The raw error is dominated by out-of-band noise (320 kHz IQ bandwidth); the loop can only
    act below ~15 kHz, so this is the number that should drop when the lock engages."""
    x = np.asarray(x, dtype=float)
    x = x - np.mean(x)
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), d=duration / len(x))
    X[f > fc] = 0.0
    return float(np.sqrt(np.mean(np.fft.irfft(X, n=len(x)) ** 2)))


# ---------------------------------------------------------------- commands
def cmd_sweep():
    """Open loop: drive on, servo off, triangle on OUT1 -> fringes on iq0."""
    p, rp = connect()
    setup_drive_and_demod(rp)
    pid = setup_pid(rp)
    pid.output_direct = 'off'
    assert abs(pid.max_voltage - OUT1_LIMIT) < 0.02, f"clamp readback {pid.max_voltage} != {OUT1_LIMIT}: not sweeping"
    sweep_on(rp)
    time.sleep(0.5)
    err, out = scope_curve(rp, duration=2.0 / SWEEP_FREQ)
    vmax, vmin = float(np.nanmax(err)), float(np.nanmin(err))
    print(f"fringe on iq0 quadrature:  max = {vmax:+.4f}   min = {vmin:+.4f}   "
          f"midpoint = {(vmax + vmin) / 2:+.4f}   amplitude = {(vmax - vmin) / 2:.4f}")
    print("Expect a clean sinusoid vs. sweep voltage, one period per 102 kHz of laser tuning.")
    print("If the amplitude is < ~0.05, raise IQ_QFACTOR (or optical power / less IN1 pad).")
    print("If the midpoint is not ~0, set pid.setpoint to the midpoint (digital IQ should be ~0).")
    print("Sweep is left running.  Run 'lock' to close the loop, 'unlock' to stop.")


def cmd_calib():
    """Actuator calibration by fringe counting with a FAST sweep so the sweep's
    fringes outrun the laser's own noise crossings. 0 V run = noise baseline.
    One fringe = 1/tau_d = 102 kHz exactly (2 km spool)."""
    SWEEP_HZ   = 100.0
    p, rp = connect()
    setup_drive_and_demod(rp)
    pid = setup_pid(rp)
    pid.output_direct = 'off'
    base = None
    for ampl in (0.0, min(0.5, 50e6 / K_ACT_OUT1), min(1.0, 100e6 / K_ACT_OUT1)):   # 0, ~50 MHz, ~100 MHz of sweep (capped at the DAC range)
        asg = rp.asg1
        asg.setup(waveform='ramp', frequency=SWEEP_HZ, amplitude=ampl, offset=0.0,
                  output_direct='out1', trigger_source='immediately')
        time.sleep(0.3)
        counts = []
        for _ in range(5):
            err, ramp = scope_curve(rp, ch1='iq0', ch2='asg1', duration=1.0 / SWEEP_HZ)
            err = err - np.mean(err)
            n = len(err) // 2
            counts.append(int(np.sum(np.abs(np.diff(np.sign(err[:n]))) > 0)))
        c = float(np.median(counts))
        if ampl == 0.0:
            base = c
            print(f"baseline (no sweep): {c:.0f} crossings per {500/SWEEP_HZ:.1f} ms  "
                  f"= {c / (0.5 / SWEEP_HZ) / 1e3:.1f} kHz noise crossing rate")
            continue
        fringes = max(c - base, 0.0) / 2.0
        volts = 2.0 * ampl
        gain = fringes * FRINGE_HZ / volts
        print(f"ampl = {ampl:.2f} V: {c:.0f} crossings (-{base:.0f} baseline) -> {fringes:.0f} fringes "
              f"over {volts:.1f} V => {gain / 1e6:.2f} MHz/V (ref. OUT1)")
    print("Measured 2026-09-16 with the 47 kOhm divider: ~4-6 MHz/V (PLANEX ~30 MHz/mA), not the 30 MHz/V design value.")
    sweep_off(rp)


def cmd_lock():
    """Close the loop: P-only, ramp P, then add the integrator."""
    p, rp = connect()
    setup_drive_and_demod(rp)
    sweep_off(rp)
    pid = setup_pid(rp, p=P_START, i=0.0)
    assert abs(pid.max_voltage - OUT1_LIMIT) < 0.02, f"clamp readback {pid.max_voltage} != {OUT1_LIMIT}: NOT closing the loop"
    if pid.p == 0.0:
        print(f"pid0.p reads back 0 after setting {P_START:.2e}: the PID register cannot resolve this gain. "
              f"Use the divider (DIVIDER = 0.01). NOT closing the loop.")
        return
    print(f"P readback = {pid.p:.3e} (requested {P_START:.3e}), clamp +/-{pid.max_voltage:.2f} V")
    pid.output_direct = 'out1'
    print(f"loop closed, P = {pid.p:g}, I = 0  (watch the 40 MHz line on the analyzer)")
    time.sleep(1.0)

    for pval in P_STEPS:
        pid.p = pval
        time.sleep(1.0)
        err, out = scope_curve(rp, duration=0.05)
        print(f"P = {pval:<8g}  err RMS = {np.std(err):.4f}   OUT1 mean = {np.mean(out):+.3f} V   "
              f"OUT1 RMS = {np.std(out):.4f} V")
        ans = input("   line still narrow? [Enter = raise P again / b = back off & continue / q = quit] ").strip().lower()
        if ans == 'q':
            return
        if ans == 'b':
            pid.p = pval / 2.0
            print(f"   backed off to P = {pid.p:g}")
            break

    pid.ival = 0.0
    pid.i = I_UNITY_HZ
    time.sleep(1.0)
    err, out = scope_curve(rp, duration=0.5)
    print(f"integrator on: I = {I_UNITY_HZ:g} Hz   err RMS = {np.std(err):.4f} (raw)   "
          f"err RMS < 5 kHz = {lf_rms(err, 0.5):.4f}   OUT1 mean = {np.mean(out):+.3f} V (drifts slowly with the laser)")
    print("Judge the lock on the < 5 kHz number and the analyzer carrier; the raw RMS is out-of-band noise.")
    print("Locked if the error trace is flat at the setpoint and the analyzer line is unchanged.")
    print("Nudge the CLD1015 temperature set point when OUT1 approaches +/-1 V.")


def cmd_unlock():
    p, rp = connect()
    safe_outputs(rp)
    print(f"loop open: OUT1 off, integrator zeroed, gains zeroed, clamp +/-{OUT1_LIMIT:g} V; drive on OUT2 left running")


def cmd_status():
    p, rp = connect()
    iq, pid = rp.iq0, rp.pid0               # READ-ONLY: does not touch the loop (safe to call while locked)
    print(f"config: DIVIDER = {DIVIDER:g}  K_act(OUT1) = {K_ACT_OUT1/1e6:.1f} MHz/V  OUT1 clamp = +/-{OUT1_LIMIT:g} V  "
          f"fringe = {102e3 / K_ACT_OUT1 * 1e3:.3f} mV = {102e3 / K_ACT_OUT1 / 0.244e-3:.2f} LSB   "
          f"P_START = {P_START:.2e}  I = {I_UNITY_HZ:.3g} Hz")
    print(f"iq0 : f = {iq.frequency/1e6:.6f} MHz  ampl = {iq.amplitude:.3f} V  bw = {iq.bandwidth}  "
          f"acbw = {iq.acbandwidth:g}  qf = {iq.quadrature_factor:g}  out = {iq.output_direct}")
    print(f"pid0: p = {pid.p:g}  i = {pid.i:g} Hz  setpoint = {pid.setpoint:+.4f}  "
          f"limits = [{pid.min_voltage:+.2f}, {pid.max_voltage:+.2f}] V  out = {pid.output_direct}"
          f"   <-- LIVE registers; if out = out1 the loop is closed")
    err, out = scope_curve(rp, duration=0.2)
    print(f"err RMS = {np.std(err):.4f} (raw)   err RMS < 5 kHz = {lf_rms(err, 0.2):.4f}   "
          f"OUT1 mean = {np.mean(out):+.3f} V   OUT1 RMS = {np.std(out):.4f} V")


def cmd_shell():
    """One persistent connection; commands as functions. No reconnects, so nothing
    is reloaded from the config file between calls."""
    import code
    p, rp = connect()
    iq, pid = rp.iq0, rp.pid0

    def status():
        """READ-ONLY. Lock quality = 'err RMS < 5 kHz' against the unlocked fringe RMS (ampl/sqrt(2))
        from the last sweep(): locked well < 0.02 V (> 12 dB), slipping ~ the unlocked value."""
        err, out = scope_curve(rp, duration=0.2)
        e_lf = lf_rms(err, 0.2)
        print(f"pid0: p = {pid.p:g}  i = {pid.i:g} Hz  ival = {pid.ival:+.3f}  limits = [{pid.min_voltage:+.2f}, {pid.max_voltage:+.2f}]  out = {pid.output_direct}")
        line = (f"err mean = {np.mean(err):+.4f}   err RMS < 5 kHz = {e_lf:.4f}   "
                f"OUT1 mean = {np.mean(out):+.3f} V   OUT1 RMS = {np.std(out):.4f} V")
        a = LAST_FRINGE['ampl']
        if a and pid.output_direct == 'out1':
            unl = a / np.sqrt(2.0)
            line += f"   suppression = {20*np.log10(unl / max(e_lf, 1e-6)):+.1f} dB (vs. unlocked {unl:.4f})"
        print(line)
        if abs(pid.ival) > 0.95 * OUT1_LIMIT and pid.output_direct == 'out1':
            print("   ** OUT1 at the rail: integrator wound up -> recenter() (and nudge the CLD1015 TEC set point)")

    def unlock():
        safe_outputs(rp)
        print("loop open")

    def sweep():
        setup_drive_and_demod(rp); setup_pid(rp); sweep_on(rp); time.sleep(0.5)
        err, _ = scope_curve(rp, duration=2.0 / SWEEP_FREQ)
        ampl = (np.nanmax(err) - np.nanmin(err)) / 2
        LAST_FRINGE['ampl'] = float(ampl)
        print(f"fringe: max {np.nanmax(err):+.4f}  min {np.nanmin(err):+.4f}  ampl {ampl:.4f}   "
              f"(unlocked RMS = {ampl/np.sqrt(2):.4f}; status() reports suppression against this)")

    def errscope(duration=0.02, save='errscope.png'):
        """Time-domain look at the raw error: LOCKED = flat noisy line near 0 (all samples well
        inside +/-0.7 A); SLIPPING = sinusoids sweeping the full +/-A. Prints the slip fraction and
        saves a plot. Read-only."""
        err, out = scope_curve(rp, duration=duration)
        a = LAST_FRINGE['ampl'] or float(np.max(np.abs(err)))
        frac = float(np.mean(np.abs(err) > 0.7 * a))
        print(f"{duration*1e3:.0f} ms of error: RMS = {np.std(err):.4f}  mean = {np.mean(err):+.4f}  "
              f"peak = {np.max(np.abs(err)):.4f}  fringe A = {a:.4f}  "
              f"samples beyond 0.7 A = {100*frac:.1f} %  -> {'LOCKED' if frac < 0.02 else 'SLIPPING'}")
        # fringe-crossing rate: 2 zero crossings per fringe; one fringe = FRINGE_HZ of laser tuning
        e0 = err - np.mean(err)
        nx = int(np.sum(np.abs(np.diff(np.sign(e0))) > 0))
        rate = nx / 2.0 / duration
        print(f"zero crossings: {nx}  -> {rate/1e3:.2f} kfringes/s  = {rate*FRINGE_HZ/1e9:.3f} GHz/s of frequency slew (if monotonic)")
        # dominant spectral line of the error (an NCO/beat OFFSET shows as one tone carrying most of the power)
        X = np.abs(np.fft.rfft(e0 * np.hanning(len(e0)))) ** 2
        f = np.fft.rfftfreq(len(e0), d=duration / len(e0))
        X[0] = 0.0
        k = int(np.argmax(X))
        band = X[max(k-2, 0):k+3].sum() / max(X.sum(), 1e-30)
        print(f"dominant tone: {f[k]/1e3:.2f} kHz carrying {100*band:.0f} % of the error power  "
              f"({'single tone -> beat/NCO offset?' if band > 0.5 else 'broadband -> real noise'})")
        try:
            import matplotlib; matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            t = np.linspace(0, duration * 1e3, len(err))
            fig, ax = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
            ax[0].plot(t, err, lw=0.6); ax[0].axhline(+a, ls='--', c='0.5'); ax[0].axhline(-a, ls='--', c='0.5')
            ax[0].set_ylabel('Error (iq0) [V]'); ax[0].set_title('Raw Error Signal And Servo Output')
            ax[1].plot(t, out, lw=0.6, c='C1'); ax[1].set_ylabel('OUT1 [V]'); ax[1].set_xlabel('Time [ms]')
            for x in ax: x.grid(alpha=0.3)
            fig.tight_layout(); fig.savefig(save, dpi=150); plt.close(fig)
            print(f"saved {save}")
        except Exception as ex:
            print(f"(no plot: {ex})")

    def lock(pgain=None, ihz=None):
        """P_START / I_UNITY_HZ assume a 0.16 V fringe. Scale both by 0.16/A from the last sweep()
        so L_P and the crossover stay at their design values whatever the visibility (A = 0.40
        after re-peaking the paddles on 2026-09-28 would otherwise put the crossover at ~30 kHz)."""
        a = LAST_FRINGE['ampl']
        scale = (0.16 / a) if a else 1.0
        if a and abs(scale - 1) > 0.2:
            print(f"fringe A = {a:.3f} (design 0.16): scaling P and I by {scale:.2f}")
        pg = P_START * scale if pgain is None else pgain
        ih = I_UNITY_HZ * scale if ihz is None else ihz
        setup_drive_and_demod(rp)
        setup_pid(rp, p=pg, i=0.0)
        pid.output_direct = 'out1'
        time.sleep(0.5)
        pid.ival = 0.0
        pid.i = ih
        time.sleep(1.0)
        print(f"closed: P = {pid.p:g}  I = {pid.i:g} Hz")
        status()

    def seti(ihz):
        """change the integrator on the fly (loop stays closed). Capped at I_MAX_HZ: through the
        100:1 plant gain, I = 25 Hz is already a ~12 kHz crossover; 100 Hz hits the delay line's
        -180 deg point (51 kHz) and the loop rings, slips fringes and the integrator ramps to a rail."""
        a = LAST_FRINGE['ampl']
        cap = I_MAX_HZ * ((0.16 / a) if a else 1.0)
        if ihz > cap:
            print(f"seti({ihz}) refused: cap is {cap:.3g} Hz for fringe A = {a or 0.16:.3f} "
                  f"(crossover ~{ihz * (a or 0.16) * 6.16e-5 * K_ACT_OUT1 / 1e3:.0f} kHz)")
            return
        pid.i = ihz; time.sleep(1.0); status()

    def setp(pgain):
        pid.p = pgain; time.sleep(1.0); status()

    def recenter():
        """zero the integrator (use right after a TEC nudge brought OUT1 back near 0)"""
        pid.ival = 0.0; time.sleep(0.5); status()

    banner = ("PyRPL shell.  status()  sweep()  lock()  lock(p, ihz)  seti(hz)  setp(p)  recenter()  errscope()  unlock()   "
              "rp/iq/pid are live objects.  Ctrl-Z Enter (Windows) or exit() to leave; the loop stays as you left it.\n"
              "Run sweep() once first so status() can report suppression [dB] against the fringe.")
    code.interact(banner=banner, local=dict(rp=rp, iq=iq, pid=pid, status=status, unlock=unlock, sweep=sweep,
                                             lock=lock, seti=seti, setp=setp, recenter=recenter, errscope=errscope,
                                             np=np, time=time))


if __name__ == "__main__":
    cmds = {'sweep': cmd_sweep, 'lock': cmd_lock, 'unlock': cmd_unlock, 'status': cmd_status, 'calib': cmd_calib, 'shell': cmd_shell}
    if len(sys.argv) != 2 or sys.argv[1] not in cmds:
        print(__doc__)
        sys.exit(1)
    cmds[sys.argv[1]]()
