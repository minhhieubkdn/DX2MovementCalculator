import pandas as pd
from scipy.spatial import KDTree
from serial import *
import time
import math
# Đọc dữ liệu
data_path = 'datatest.csv'
data = pd.read_csv(data_path)

# Tạo KDTree để tìm kiếm nhanh các điểm lân cận
features = data[['begin_velocity', 'desired_velocity', 'acceleration', 'distance_to_move']]
tree = KDTree(features)

def predict_execute_time_knn(begin_velocity, desired_velocity, acceleration, distance_to_move, k=2):
    """
    Dự đoán execute_time dựa trên k điểm lân cận nhất.
    
    Parameters:
    - begin_velocity, desired_velocity, acceleration, distance_to_move: các tham số đầu vào.
    - k: Số lượng điểm lân cận để xem xét (mặc định là 5).
    
    Returns:
    - Trung bình execute_time của k điểm lân cận nhất.
    """
    # Tìm k điểm lân cận nhất
    distances, indices = tree.query([begin_velocity, desired_velocity, acceleration, distance_to_move], k=k)
    
    # Tính trung bình execute_time của các điểm này
    average_time = data.loc[indices, 'execute_time'].mean()
    return average_time

ser = Serial(port='COM10', baudrate=115200, timeout=1)

# Đợi 2 giây để Arduino khởi động
time.sleep(2)

def send_gcode_command(ser, command):
    ser.write(command.encode())
    # print(f"Đã gửi lệnh: {command}")

    response = ""
    while "Ok" not in response:
        response = ser.readline().decode()
        # print(f"Đã nhận phản hồi: {response}")

def calculate_movement_time(v_start, v_max, a_max, distance):
    # Tính thời gian tăng tốc
    t_acc = (v_max - v_start) / a_max
    d_acc = v_start * t_acc + 0.5 * a_max * t_acc**2
    t_dec = (v_max - v_start) / a_max
    d_dec = v_max * t_dec - 0.5 * a_max * t_dec**2
    d_max = distance - d_acc - d_dec
    is_reached_vmax = False
    if d_max < 0:
        # Không đạt vận tốc lớn nhất
        t_max = 0
        t_acc = t_dec = predict_execute_time_knn(v_start, v_max, a_max, distance, 2) / 2000
    else:
        is_reached_vmax = True
        t_max = d_max / v_max

    # Tính tổng thời gian di chuyển
    total_time = t_acc + t_max + t_dec
    return total_time * 1000 , is_reached_vmax

def get_real_execute_time(v_start, v_max, a_max, distance):
    send_gcode_command(ser, "G28\n")
    send_gcode_command(ser, "M204 A{}\n".format(5000))
    send_gcode_command(ser, "M205 S{}\n".format(100))
    send_gcode_command(ser, "G01 X{} Y0 Z-325 F{}\n".format(distance/2, 500))

    start_time = time.time()
    send_gcode_command(ser, "M204 A{}\n".format(a_max))
    send_gcode_command(ser, "M205 S{}\n".format(v_start))
    send_gcode_command(ser, "G01 X{} Y0 Z-325 F{}\n".format(-distance/2, v_max))
    end_time = time.time()
    execution_time = end_time - start_time
    return execution_time * 1000


# 100,800,1000,320,1339.350938796997
a, f = calculate_movement_time(5, 800, 1000, 150)
b = get_real_execute_time(5, 800, 1000, 150)
print("cal_time: {}, real_time: {}, delta: {}, is_reach_max: {}".format(a, b, b-a, f))