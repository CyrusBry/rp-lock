# rp-lock — PLANEX Delay-Line Lock On A Red Pitaya (UCLA Build 2)

PyRPL script that closes the self-heterodyne 2 km fiber delay-line lock of a RIO PLANEX
with a Red Pitaya STEMlab 125-14 (PyRPL, Red Pitaya fork). Companion to `UCLA Build.pdf`.

## Setup (once per computer)
```
conda create -n pyrpl-env python=3.11
conda activate pyrpl-env
pip install pyrpl numpy matplotlib
```
Static IP of the board is set in `HOSTNAME` (lab PC direct link: `192.168.10.2`).

## Use
```
python build1a_pyrpl.py shell
```
Then at the `>>>` prompt: `sweep()` (once, records the fringe) → `lock()` → `status()` → `errscope()` → `unlock()`.
`seti()` is capped at 40 Hz (the 100:1 plant gain puts I = 25 Hz at a ~12 kHz crossover).

## Bench state
See the header of `build1a_pyrpl.py` (signal chain, divider, measured actuator gain) and the dated
comments in the settings block. Diagnostics saved by `errscope()` land next to the script.
