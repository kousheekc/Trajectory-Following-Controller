import time
import math
import serial
import numpy as np
import matplotlib.pyplot as plt


def plot_via_points(points, waypoints):
    plt.xlim(-0.5, 1.5)
    plt.ylim(-0.5, 1.5)
    plt.gca().set_aspect('equal', adjustable='box')
    
    plt.scatter(points[:, 0], points[:, 1])
    # plt.scatter(points[:, 2], points[:, 3])
    # plt.scatter(points[:, 4], points[:, 5])

    for point in points:
        # plt.arrow(point[0], point[1], point[2] - point[0], point[3] - point[1], width = 0.05)
        plt.arrow(point[0], point[1], point[4] - point[0], point[5] - point[1], width = 0.005)

    plt.plot(waypoints[:, 0], waypoints[:, 1])    
    plt.show()
    return waypoints

def generate_waypoints(coeffs, k):
    all_x = np.array([])
    all_y = np.array([])
    all_theta = np.array([])

    t = np.linspace(0,1,k)
    for coeff in coeffs:
        x = coeff[0] + coeff[1]*t + coeff[2]*t**2 + coeff[3]*t**3   
        y = coeff[4] + coeff[5]*t + coeff[6]*t**2 + coeff[7]*t**3 
        dxdt = coeff[1] + 2*coeff[2]*t + 3*coeff[3]*t**2
        dydt = coeff[5] + 2*coeff[6]*t + 3*coeff[7]*t**2
        theta = np.arctan2(dydt, dxdt)
        all_x = np.append(all_x, x)
        all_y = np.append(all_y, y)
        all_theta = np.append(all_theta, theta)

    waypoints = np.column_stack((all_x, all_y, all_theta))

    return waypoints

def generate_velocity_control_signals(coeff, a, t):
    x_dot = (a**2*coeff[1] + 2*a*coeff[2]*t + 3*coeff[3]*t**2)/(a**3)
    x_dot_times_a_cube = (a**2*coeff[1] + 2*a*coeff[2]*t + 3*coeff[3]*t**2)
    y_dot = (a**2*coeff[5] + 2*a*coeff[6]*t + 3*coeff[7]*t**2)/(a**3)
    y_dot_times_a_cube = (a**2*coeff[5] + 2*a*coeff[6]*t + 3*coeff[7]*t**2)

    x_double_dot = (2*a*coeff[2] + 6*coeff[3]*t)/(a**3)
    x_double_dot_times_a_cube = (2*a*coeff[2] + 6*coeff[3]*t)
    y_double_dot = (2*a*coeff[6] + 6*coeff[7]*t)/(a**3)
    y_double_dot_times_a_cube = (2*a*coeff[6] + 6*coeff[7]*t)

    v_dot = math.sqrt(x_dot**2 + y_dot**2)
    w_dot = (x_dot_times_a_cube * y_double_dot_times_a_cube - y_dot_times_a_cube * x_double_dot_times_a_cube) / (x_dot_times_a_cube**2 + y_dot_times_a_cube**2)

    return v_dot, w_dot


def generate_guide_points(points, k):
    new_points = []
    for point in points:
        x_minus = point[0] - k*math.cos(math.radians(point[2]))
        y_minus = point[1] - k*math.sin(math.radians(point[2]))
        x_plus = point[0] + k*math.cos(math.radians(point[2]))
        y_plus = point[1] + k*math.sin(math.radians(point[2]))
        new_point = [point[0], point[1], x_minus, y_minus, x_plus, y_plus]
        new_points.append(new_point)

    return np.array(new_points)


def bezier_curve(points):
    coeffs = []
    for i in range(len(points)-1):
        a0i = points[i][0]
        b0i = points[i][1]
        a1i = 3*(points[i][4] - points[i][0])
        b1i = 3*(points[i][5] - points[i][1])
        a2i = 3*(points[i][0] + points[i+1][2] - 2*points[i][4])
        b2i = 3*(points[i][1] + points[i+1][3] - 2*points[i][5])
        a3i = points[i+1][0] - points[i][0] + 3*points[i][4] - 3*points[i+1][2]
        b3i = points[i+1][1] - points[i][1] + 3*points[i][5] - 3*points[i+1][3]
        coeff = [a0i, a1i, a2i, a3i, b0i, b1i, b2i, b3i]
        coeffs.append(coeff)
    return np.array(coeffs)

def convert_to_command(linear, angular):
    command = f'{round(linear, 4)},{round(angular, 4)},\n'
    return command.encode('utf-8')

points = [
    [0,0,0,0],
    [0.3,0.8,135,8],
    [0.5,0.2,45,8],
    [0.8,0.9,90,8]
]

points = np.array(points)
points_with_guides = generate_guide_points(points, 0.5)
coeffs = bezier_curve(points_with_guides)

waypoints = generate_waypoints(coeffs, 100)
plot_via_points(points_with_guides, waypoints)

# # ser = serial.Serial('COM4')

# for i in range(1, len(points)):
#     init_time = time.perf_counter()
#     while (time.perf_counter() - init_time <= points[i][3]):
#         linear_velocity, angular_velocity = generate_velocity_control_signals(coeffs[i-1],  points[i][3], (time.perf_counter() - init_time))
#         velocity_command = convert_to_command(linear_velocity, angular_velocity)
#         print(round((time.perf_counter() - init_time), 2), velocity_command)
#         # ser.write(velocity_command)


# linear_velocity, angular_velocity = 0, 0
# velocity_command = convert_to_command(linear_velocity, angular_velocity)
# print(velocity_command)
# # ser.write(velocity_command)