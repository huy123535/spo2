import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque
import time

# Cấu hình Serial
SERIAL_PORT = 'COM5'  # Thay đổi port COM phù hợp với máy của bạn
BAUD_RATE = 115200

# Cấu hình đồ thị
MAX_POINTS = 1000  # Số điểm tối đa hiển thị trên đồ thị
PLOT_INTERVAL = 20  # Thời gian cập nhật đồ thị (ms)

# Khởi tạo dữ liệu
red_raw = deque(maxlen=MAX_POINTS)
ir_raw = deque(maxlen=MAX_POINTS)
red_filtered = deque(maxlen=MAX_POINTS)
ir_filtered = deque(maxlen=MAX_POINTS)
red_diff = deque(maxlen=MAX_POINTS)
timestamps = deque(maxlen=MAX_POINTS)

# Khởi tạo đồ thị
plt.style.use('dark_background')
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 8))
fig.suptitle('MAX30102 Sensor Data', fontsize=16)

# Cấu hình các subplot
ax1.set_title('Raw Signals')
ax1.set_ylabel('Amplitude')
ax1.grid(True)

ax2.set_title('Filtered Signals')
ax2.set_ylabel('Amplitude')
ax2.grid(True)

ax3.set_title('Red Signal Derivative')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Amplitude')
ax3.grid(True)

# Khởi tạo các đường
line1_red, = ax1.plot([], [], 'r-', label='Red Raw')
line1_ir, = ax1.plot([], [], 'g-', label='IR Raw')
line2_red, = ax2.plot([], [], 'r-', label='Red Filtered')
line2_ir, = ax2.plot([], [], 'g-', label='IR Filtered')
line3, = ax3.plot([], [], 'b-', label='Red Diff')

# Thêm legend
ax1.legend()
ax2.legend()
ax3.legend()

# Hàm khởi tạo animation
def init():
    for line in [line1_red, line1_ir, line2_red, line2_ir, line3]:
        line.set_data([], [])
    return line1_red, line1_ir, line2_red, line2_ir, line3

# Hàm cập nhật animation
def update(frame):
    try:
        # Đọc dữ liệu từ Serial
        if ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()
            if ',' in line:
                red, ir, red_filt, ir_filt, diff = map(float, line.split(','))
                
                # Cập nhật dữ liệu
                current_time = time.time()
                timestamps.append(current_time)
                red_raw.append(red)
                ir_raw.append(ir)
                red_filtered.append(red_filt)
                ir_filtered.append(ir_filt)
                red_diff.append(diff)

                # Cập nhật đồ thị
                t = np.array(timestamps) - timestamps[0]
                
                line1_red.set_data(t, red_raw)
                line1_ir.set_data(t, ir_raw)
                line2_red.set_data(t, red_filtered)
                line2_ir.set_data(t, ir_filtered)
                line3.set_data(t, red_diff)

                # Tự động điều chỉnh trục
                for ax in [ax1, ax2, ax3]:
                    ax.relim()
                    ax.autoscale_view()

    except Exception as e:
        print(f"Error: {e}")

    return line1_red, line1_ir, line2_red, line2_ir, line3

# Kết nối Serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    print(f"Connected to {SERIAL_PORT}")
    
    # Tạo animation
    ani = FuncAnimation(fig, update, init_func=init, interval=PLOT_INTERVAL, blit=True)
    plt.tight_layout()
    plt.show()

except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
finally:
    if 'ser' in locals():
        ser.close() 