import serial
import matplotlib.pyplot as plt

# Cấu hình cổng COM và baudrate (thay COM3 bằng cổng của bạn)
ser = serial.Serial('COM5', 115200, timeout=1)

x_data = []
y_data = []

plt.ion()  # Bật chế độ interactive
fig, ax = plt.subplots()
line, = ax.plot([], [], marker='o')
ax.set_xlabel('x')
ax.set_ylabel('Red Value')
ax.set_title('MAX30101 Real-time Data')
ax.grid(True)

while True:
    try:
        line_bytes = ser.readline()
        line_str = line_bytes.decode('utf-8').strip()
        if ',' in line_str:
            x_str, y_str = line_str.split(',')
            x = int(x_str)
            y = int(y_str)
            x_data.append(x)
            y_data.append(y)
            line.set_data(x_data, y_data)
            ax.relim()
            ax.autoscale_view()
            plt.xticks(range(0, max(x_data)+1, 100))  # Vạch x cách nhau 100
            plt.draw()
            plt.pause(0.01)
    except KeyboardInterrupt:
        print('Stopped by user')
        break
    except Exception as e:
        print('Error:', e)

ser.close()