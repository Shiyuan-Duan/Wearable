import asyncio
import time
import numpy as np
import matplotlib.pyplot as plt
from bleak import BleakScanner, BleakClient

# ---------------------------------
# BLE UUIDs (in standard 128-bit format)
# ---------------------------------
SERVICE_UUID = "a0a43180-96be-4222-b41e-98ea76b0120c"
ECG_UUID     = "a0a43181-96be-4222-b41e-98ea76b0120c"  # Channel 1
BIOZ_UUID    = "a0a43182-96be-4222-b41e-98ea76b0120c"  # Channel 2

# ---------------------------------
# Sampling rates & ring-buffer sizes (assumed)
# ---------------------------------
ECG_SPS       = 512
BIOZ_SPS      = 64

# Keep 3 seconds of data
ECG_BUF_SIZE  = 3 * ECG_SPS     # 3 sec * 512 SPS
BIOZ_BUF_SIZE = 3 * BIOZ_SPS    # 3 sec * 64 SPS

# ---------------------------------
# Global buffers (32-bit) to hold streaming data
# ---------------------------------
ecg_data  = np.zeros(ECG_BUF_SIZE, dtype=np.int32)
bioz_data = np.zeros(BIOZ_BUF_SIZE, dtype=np.int32)

# Indices for the circular buffers
ecg_index  = 0
bioz_index = 0

# For plotting time (static array from -3s to 0s)
ecg_time  = np.linspace(-3, 0, ECG_BUF_SIZE)
bioz_time = np.linspace(-3, 0, BIOZ_BUF_SIZE)

# ---------------------------------
# Counters to confirm sampling rate
# ---------------------------------
ecg_sample_count  = 0
bioz_sample_count = 0
last_log_time = time.time()

# ---------------------------------
# Notification handlers
# ---------------------------------
def ecg_notification_handler(sender: str, data: bytearray):
    """
    Callback for ECG characteristic notifications.
    Expects multiple 32-bit signed samples (int32_t) per notification.
    For example, if len(data) = 12, that implies 3 samples.
    """
    global ecg_data, ecg_index, ecg_sample_count

    # Parse each 4-byte chunk as one int32_t
    for i in range(0, len(data), 4):
        sample_bytes = data[i : i + 4]
        value = int.from_bytes(sample_bytes, byteorder='little', signed=True)
        # Store in the ring buffer
        ecg_data[ecg_index] = value
        ecg_index = (ecg_index + 1) % ECG_BUF_SIZE

        ecg_sample_count += 1


def bioz_notification_handler(sender: str, data: bytearray):
    """
    Callback for BIOZ characteristic notifications.
    Expects multiple 32-bit signed samples (int32_t) per notification.
    """
    global bioz_data, bioz_index, bioz_sample_count

    # Parse each 4-byte chunk as one int32_t
    for i in range(0, len(data), 4):
        sample_bytes = data[i : i + 4]
        value = int.from_bytes(sample_bytes, byteorder='little', signed=True)
        # Store in the ring buffer
        bioz_data[bioz_index] = value
        bioz_index = (bioz_index + 1) % BIOZ_BUF_SIZE

        bioz_sample_count += 1

# ---------------------------------
# BLE Connection + Subscription (async)
# ---------------------------------
async def ble_task():
    """
    Background task that:
    1) Scans for the device named "ECG".
    2) Connects and subscribes to ECG + BIOZ notifications.
    3) Remains connected, letting notifications flow indefinitely.
    """
    print("Scanning for BLE devices named 'ECG'...")
    devices = await BleakScanner.discover(timeout=5.0)

    target = None
    for d in devices:
        if d.name == "ECG":
            target = d
            break

    if not target:
        print("ECG device not found. Exiting.")
        return

    print(f"Found device: {target.name} [{target.address}]")
    print("Connecting...")

    async with BleakClient(target.address) as client:
        # Check if the service is present
        services = await client.get_services()
        if SERVICE_UUID not in [s.uuid for s in services]:
            print("Warning: ECG Service UUID not found on this device.")

        # Subscribe to ECG characteristic
        await client.start_notify(ECG_UUID, ecg_notification_handler)
        print("Subscribed to ECG characteristic.")

        # Subscribe to BIOZ characteristic
        await client.start_notify(BIOZ_UUID, bioz_notification_handler)
        print("Subscribed to BIOZ characteristic.")

        print("BLE task is now waiting for notifications...")
        while True:
            await asyncio.sleep(1.0)

# ---------------------------------
# Plot Setup (real-time)
# ---------------------------------
plt.ion()
fig, (ax_ecg, ax_bioz) = plt.subplots(2, 1, figsize=(8, 6))
fig.suptitle("Real-Time ECG & BIOZ Streams (last 3 s)")

ecg_line, = ax_ecg.plot(ecg_time, ecg_data, label="ECG")
ax_ecg.set_xlabel("Time (s)")
ax_ecg.set_ylabel("ECG (ADC counts)")
ax_ecg.grid(True)

bioz_line, = ax_bioz.plot(bioz_time, bioz_data, label="BIOZ", color="orange")
ax_bioz.set_xlabel("Time (s)")
ax_bioz.set_ylabel("BIOZ (ADC counts)")
ax_bioz.grid(True)

plt.tight_layout()

def update_plot():
    """
    1) Roll the ring buffer so the most recent sample is at the end.
    2) Update x/y data for both ECG and BIOZ lines.
    3) Dynamically adjust x and y limits.
    """
    global ecg_data, bioz_data, ecg_index, bioz_index

    # Roll to align newest sample at the right
    ecg_rolled = np.roll(ecg_data, -ecg_index)
    bioz_rolled = np.roll(bioz_data, -bioz_index)

    # Update line data
    ecg_line.set_xdata(ecg_time)
    ecg_line.set_ydata(ecg_rolled)

    bioz_line.set_xdata(bioz_time)
    bioz_line.set_ydata(bioz_rolled)

    # X-axis fixed from -3..0
    ax_ecg.set_xlim(ecg_time[0], ecg_time[-1])
    ax_bioz.set_xlim(bioz_time[0], bioz_time[-1])

    # Dynamic y-limits
    # ECG
    ecg_min, ecg_max = ecg_rolled.min(), ecg_rolled.max()
    ecg_range = ecg_max - ecg_min
    if ecg_range == 0:
        ecg_min -= 100
        ecg_max += 100
    else:
        margin = 0.1 * ecg_range
        ecg_min -= margin
        ecg_max += margin
    ax_ecg.set_ylim(ecg_min, ecg_max)

    # BIOZ
    bioz_min, bioz_max = bioz_rolled.min(), bioz_rolled.max()
    bioz_range = bioz_max - bioz_min
    if bioz_range == 0:
        bioz_min -= 100
        bioz_max += 100
    else:
        margin = 0.1 * bioz_range
        bioz_min -= margin
        bioz_max += margin
    ax_bioz.set_ylim(bioz_min, bioz_max)

    plt.draw()
    plt.pause(0.001)

async def main():
    global ecg_sample_count, bioz_sample_count, last_log_time

    # Start the BLE background task
    ble_future = asyncio.create_task(ble_task())

    # Update the plot in the foreground ~ every 100 ms
    while True:
        current_time = time.time()
        # Print sample counts every 1 second
        if (current_time - last_log_time) >= 1.0:
            # Approximate the sample rate
            elapsed = current_time - last_log_time

            ecg_rate  = ecg_sample_count  / elapsed
            bioz_rate = bioz_sample_count / elapsed

            print(f"[{time.strftime('%H:%M:%S')}] "
                  f"ECG samples: {ecg_sample_count} (~{ecg_rate:.1f} Hz), "
                  f"BIOZ samples: {bioz_sample_count} (~{bioz_rate:.1f} Hz)")

            # Reset counters
            ecg_sample_count  = 0
            bioz_sample_count = 0
            last_log_time = current_time

        # Update plot
        update_plot()

        # Sleep a bit
        await asyncio.sleep(0.1)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Exiting on Ctrl+C")
