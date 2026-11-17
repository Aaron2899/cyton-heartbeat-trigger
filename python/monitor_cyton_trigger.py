"""
Check that cyton_rwave_trigger firmware is working.

Streams for N seconds, then reports detected beats, triggers, heart rate and
where each trigger landed relative to its beat, and saves a plot.

    pip install brainflow numpy matplotlib
    python monitor_cyton_trigger.py --port COM3 --seconds 60 --channel 1
"""
import argparse
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from brainflow.board_shim import BoardShim, BrainFlowInputParams, BoardIds

ap = argparse.ArgumentParser()
ap.add_argument("--port", required=True)
ap.add_argument("--seconds", type=float, default=60)
ap.add_argument("--channel", type=int, default=1)
args = ap.parse_args()

bid = BoardIds.CYTON_BOARD.value
params = BrainFlowInputParams()
params.serial_port = args.port
board = BoardShim(bid, params)
fs = BoardShim.get_sampling_rate(bid)
ecg_row = BoardShim.get_exg_channels(bid)[args.channel - 1]
beat_row, trig_row, filt_row = BoardShim.get_analog_channels(bid)   # aux0, aux1, aux2

board.prepare_session()
for ch in "12345678":
    if ch == str(args.channel):
        board.config_board(f"x{ch}060100X")   # chosen channel: on, gain 24, normal input, bias on, SRB2 OFF, SRB1 off
    else:   
        board.config_board(f"x{ch}161000X")   # rest of the channels: off, gain 24, input shorted, bias off, SRB2 OFF, SRB1 off
board.start_stream()            # sends 'b' -> firmware starts detecting
print(f"Recording {args.seconds:.0f} s ... (detector needs ~3 s to learn)")
time.sleep(args.seconds)
data = board.get_board_data()
board.stop_stream()
board.release_session()

# ---- sample counter: every packet carries a counter that counts 0-255 and wraps
pkt = data[BoardShim.get_package_num_channel(bid)].astype(int)
steps = np.diff(pkt) % 256                    # 1 = next packet, 2 = one packet missing
steps[steps == 0] = 1                         # repeated counter: treat as the next packet
si = np.concatenate(([0], np.cumsum(steps)))  # true sample number of each received packet
t = si / fs                                   # time axis that counts missing packets as time

beats = np.flatnonzero(data[beat_row] == 1)
trigs = np.flatnonzero(data[trig_row] == 1)

# ---- missing packets: a 1-packet gap right before a trigger marker is the firmware
# skipping the send at the trigger; anything else was lost on the way
lost = steps - 1
gaps = np.flatnonzero(lost) + 1               # index of the first packet after each gap
is_skip = np.isin(gaps, trigs) & (lost[gaps - 1] == 1)
radio_gaps = gaps[~is_skip]
n_skip = int(is_skip.sum())
n_lost = int(lost.sum()) - n_skip

print(f"\nSamples: {data.shape[1]}  ({(si[-1] + 1) / fs:.1f} s)")
print(f"Beats detected: {len(beats)}")
if len(beats) > 1:
    rr = np.diff(si[beats]) / fs
    print(f"Heart rate: {60 / rr.mean():.1f} bpm  (RR {rr.mean() * 1000:.0f} ± {rr.std() * 1000:.0f} ms)")
print(f"Triggers: {len(trigs)}  ({100 * len(trigs) / max(len(beats), 1):.0f}% of beats)")
print(f"Skipped at triggers: {n_skip}  (expected: 1 per trigger)")
print(f"Lost packets: {n_lost}  ({100 * n_lost / (si[-1] + 1):.2f}%) in {len(radio_gaps)} gap(s)")

delays = []
for tr in trigs:
    prev = beats[beats < tr]
    if len(prev):
        delays.append((si[tr] - si[prev[-1]]) / fs * 1000)
if delays:
    d = np.array(delays)
    print(f"Trigger at R+{np.median(d):.0f} ms (range {d.min():.0f}-{d.max():.0f}; ±4 ms sample resolution)")


# ---- plot the last 10 s
sel = t > t[-1] - 10
fig, ax = plt.subplots(2, 1, figsize=(14, 6), sharex=True)
ax[0].plot(t[sel], data[ecg_row][sel], lw=0.7)
ax[0].set_ylabel("raw ch (uV)")
ax[1].plot(t[sel], data[filt_row][sel], lw=0.7, label="detector input (filtered)")
for b in beats[sel[beats]]:
    ax[1].axvline(t[b], color="g", alpha=0.5)
for tr in trigs[sel[trigs]]:
    ax[1].axvspan(t[tr] - 0.1, t[tr] + 0.2, color="orange", alpha=0.25)
    ax[1].axvline(t[tr], color="r")
for g in radio_gaps[sel[radio_gaps]]:
    ax[1].axvline(t[g], color="k", ls="--", alpha=0.6)
ax[1].set_ylabel("filtered (uV)")
ax[1].set_xlabel("time (s)")
ax[1].set_title("green = detected beat, red = trigger, orange = 100 ms baseline + 200 ms response, "
                "dashed black = lost packets")
plt.tight_layout()
plt.savefig("cyton_trigger_check.png", dpi=120)
print("\nSaved plot: cyton_trigger_check.png")
