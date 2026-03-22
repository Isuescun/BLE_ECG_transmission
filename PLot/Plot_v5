import serial
import serial.threaded
import matplotlib.pyplot as plt
import matplotlib.widgets as widgets
import numpy as np
from collections import deque
import threading

# ── Config ────────────────────────────────────────────────────────────────────
PORT         = "COM18"
BAUD         = 1000000
NUM_CHANNELS = 19
BATCH_SIZE   = 10
PLOT_WINDOW  = 1000

SYNC_A = 0xAA
SYNC_B = 0x55

PACKET_BYTES  = 2 + BATCH_SIZE * NUM_CHANNELS * 2   # 382 bytes
FRAME_PAYLOAD = PACKET_BYTES + 1                     # +1 for 0x0A

# ── Shared state (written by reader thread, read by plot thread) ──────────────
buffers       = [deque(maxlen=PLOT_WINDOW) for _ in range(NUM_CHANNELS)]
last_id       = None
total_dropped = 0
battery_text  = "Battery: --"
state_lock    = threading.Lock()

# ── Serial ────────────────────────────────────────────────────────────────────
ser = serial.Serial(PORT, BAUD, timeout=1)
print(f"Opened {PORT} at {BAUD} baud")

# ── Frame parsing ─────────────────────────────────────────────────────────────
def find_sync(ser):
    while True:
        b = ser.read(1)
        if not b:
            continue
        if b[0] == SYNC_A:
            b2 = ser.read(1)
            if b2 and b2[0] == SYNC_B:
                return True

def read_frame(ser):
    data = ser.read(FRAME_PAYLOAD)
    if len(data) < FRAME_PAYLOAD:
        return None
    packet_id = data[0] | (data[1] << 8)
    samples = [[] for _ in range(NUM_CHANNELS)]
    for t in range(BATCH_SIZE):
        for ch in range(NUM_CHANNELS):
            offset = 2 + (t * NUM_CHANNELS + ch) * 2
            sample = data[offset] | (data[offset + 1] << 8)
            samples[ch].append(sample)
    return packet_id, samples

# ── Reader thread ─────────────────────────────────────────────────────────────
# Runs in background, never blocks the plot
def reader_thread():
    global last_id, total_dropped, battery_text

    while True:
        try:
            b = ser.read(1)
            if not b:
                continue

            # Check for CMD: text response (battery reply etc.)
            # CMD responses start with 'C' (0x43)
            if b[0] == ord('C'):
                rest = ser.readline().decode(errors='ignore').strip()
                line = 'C' + rest
                if line.startswith("CMD:"):
                    response = line[4:]   # e.g. "BATTERY:87"
                    if response.startswith("BATTERY:"):
                        val = response.split(":")[1]
                        with state_lock:
                            battery_text = f"Battery: {val}%"
                        print(f"[BAT] {response}")
                    else:
                        print(f"[CMD] {response}")
                continue

            # Check for sync word
            if b[0] == SYNC_A:
                b2 = ser.read(1)
                if not b2 or b2[0] != SYNC_B:
                    continue

                # Read rest of frame
                data = ser.read(FRAME_PAYLOAD)
                if len(data) < FRAME_PAYLOAD:
                    continue

                packet_id = data[0] | (data[1] << 8)

                # Drop detection
                with state_lock:
                    if last_id is not None:
                        expected = (last_id + 1) & 0xFFFF
                        if packet_id != expected:
                            gap = (packet_id - expected) & 0xFFFF
                            total_dropped += gap
                            print(f"[DROP] id={packet_id} expected={expected} "
                                  f"gap={gap} total={total_dropped}")
                    last_id = packet_id

                    # Unpack and buffer samples
                    for t in range(BATCH_SIZE):
                        for ch in range(NUM_CHANNELS):
                            offset = 2 + (t * NUM_CHANNELS + ch) * 2
                            sample = data[offset] | (data[offset + 1] << 8)
                            buffers[ch].append(sample)

        except Exception as e:
            print(f"[READER ERR] {e}")

# ── Plot setup ────────────────────────────────────────────────────────────────
plt.ion()

# Extra space at top for battery button + label
fig = plt.figure(figsize=(10, NUM_CHANNELS * 0.6 + 1.2))
fig.suptitle("BLE stream -- 19 channels @ 1kHz", fontsize=11, y=0.99)

# Channel subplots occupy bottom portion
plot_top    = 0.93
plot_bottom = 0.04
ax_plots = []
for ch in range(NUM_CHANNELS):
    height = (plot_top - plot_bottom) / NUM_CHANNELS
    bottom = plot_bottom + (NUM_CHANNELS - 1 - ch) * height
    ax = fig.add_axes([0.08, bottom, 0.88, height * 0.85])
    ax.set_ylim(800, 3300)
    ax.set_xlim(0, PLOT_WINDOW)
    ax.set_yticks([])
    ax.set_ylabel(f"ch{ch}", fontsize=7, rotation=0, labelpad=24)
    ax.tick_params(left=False, bottom=(ch == NUM_CHANNELS - 1))
    for spine in ax.spines.values():
        spine.set_visible(False)
    if ch == NUM_CHANNELS - 1:
        ax.spines["bottom"].set_visible(True)
        ax.set_xlabel("Samples")
    ax_plots.append(ax)

lines = [ax.plot([], [], linewidth=0.7)[0] for ax in ax_plots]

# ── Battery button + label ────────────────────────────────────────────────────
ax_button = fig.add_axes([0.08, 0.955, 0.15, 0.03])
btn = widgets.Button(ax_button, "Get Battery", color="lightblue", hovercolor="deepskyblue")

ax_batlabel = fig.add_axes([0.25, 0.955, 0.5, 0.03])
ax_batlabel.axis("off")
bat_label = ax_batlabel.text(0, 0.5, battery_text, fontsize=11, va="center")

def on_battery_click(event):
    try:
        ser.write(b"GET_BATTERY\n")
        print("Sent: GET_BATTERY")
    except Exception as e:
        print(f"[ERR] Could not send command: {e}")

btn.on_clicked(on_battery_click)

# ── Start reader thread ───────────────────────────────────────────────────────
t = threading.Thread(target=reader_thread, daemon=True)
t.start()
print("Waiting for data...")

# ── Main plot loop ────────────────────────────────────────────────────────────
REDRAW_EVERY = 5
batch_count  = 0

while True:
    try:
        batch_count += 1
        if batch_count % REDRAW_EVERY == 0:
            with state_lock:
                for ch in range(NUM_CHANNELS):
                    y = list(buffers[ch])
                    x = np.arange(len(y))
                    lines[ch].set_xdata(x)
                    lines[ch].set_ydata(y)
                current_bat = battery_text

            bat_label.set_text(current_bat)
            fig.canvas.flush_events()

        plt.pause(0.05)

    except KeyboardInterrupt:
        print(f"\nStopped. Total dropped: {total_dropped}")
        ser.close()
        break
    except Exception as e:
        print(f"[ERR] {e}")
