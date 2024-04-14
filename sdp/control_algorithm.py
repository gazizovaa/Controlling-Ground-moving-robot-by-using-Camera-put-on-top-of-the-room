import cv2
import numpy as np
import matplotlib.pyplot as plt
import scipy.interpolate
from scipy.optimize import curve_fit
import time
import serial

# Open the camera (use 0 for the default camera)
cap = cv2.VideoCapture(2)
port = "COM4"  # This will be different for various devices,COM port. keyboard.is_pressed('q') == False
bluetooth = serial.Serial(port, 115200)  # Start communications with the bluetooth unit
bluetooth.flushInput()  # This gives the bluetooth a little kick
bluetooth.write((str(0) + str('/') + str(0) + str('/')).encode())  # These need to be bytes not unicode

# Declare the variables
Kx = 0.2
K_phi = 0.3
Ky = 0.15
a = 11.5 / 2
st = 0  # st and stop are for keeping data points apart from each other so that it is not too dense
# st is to allow lines_arr to append first time so that for the following
# points we can check whether it is apart from this first point
ret, frame = cap.read()


def draw_curve_with_mouse_events(event, x, y, flags, params):
    global lines_arr, stop, st

    if event == cv2.EVENT_LBUTTONDOWN and stop == 0:
        if st == 0:  # to allow lines_arr to append for the first time
            lines_arr.append((x, y))
            st = 1
        if (x - lines_arr[-1][0]) ** 2 + (
                y - lines_arr[-1][1]) ** 2 > 50 ** 2:  # to avoid data points which are too close to each other.
            lines_arr.append((x, y))




    elif event == cv2.EVENT_LBUTTONUP and stop == 0:
        if (x - lines_arr[-1][0]) ** 2 + (y - lines_arr[-1][1]) ** 2 > 50 ** 2:
            lines_arr.append((x, y))
        stop = 1  # when the curve is finished leave the drawing part

    elif event == cv2.EVENT_MOUSEMOVE and (flags and cv2.EVENT_FLAG_LBUTTON) and stop == 0:
        if (x - lines_arr[-1][0]) ** 2 + (y - lines_arr[-1][1]) ** 2 > 50 ** 2:
            lines_arr.append((x, y))


# # Create a window
cv2.namedWindow("Original Frame")
# Set the callback function
cv2.setMouseCallback("Original Frame", draw_curve_with_mouse_events)

lines_arr = []
stop = 0
ret, frame = cap.read()
while stop == 0:  # whithout loop we can't draw the curve because Moues_Events always draws just a point.
    cv2.imshow("Original Frame", frame)
    cv2.waitKey(5)
    for line_start_point in range(1, len(lines_arr)):
        cv2.circle(frame, lines_arr[-1], 5, (0, 255, 0), -1)
t = np.arange(0, 2, 0.01)
lines_arr = np.array(lines_arr)
xd = lines_arr[:, 0]
yd = lines_arr[:, 1]
# interpolation
tck, u = scipy.interpolate.splprep([xd, yd], s=0)

# evaluate the spline fits for 1000 evenly spaced distance values
xi, yi = scipy.interpolate.splev(np.linspace(0, 1, 10 * len(lines_arr)), tck)

# plot the result
# fig, ax = plt.subplots(1, 1)
# ax.plot(xd, yd, 'or')
# ax.plot(xi, yi, '-b')
x_d = xi
y_d = yi

# This was to define x_des and y_des mathematically by functions for testing
# x_d = (400*np.sin(t))
# y_d = (400*(np.cos(t) + t/3))
#
# plt.plot(x_d, y_d)
# plt.show()


# Identify x and y data points
x_desired = x_d
y_desired = y_d
# print(x_d)
phi = 0
phi_desired = np.arctan2(y_desired[1:] - y_desired[:-1], x_desired[1:] - x_desired[:-1])
Ts = 0.1
# Calculate the desired velocity based on the derivatives of x and y coordinated values
x_derivative = np.gradient(x_desired) / Ts
y_derivative = np.gradient(y_desired) / Ts
# v_desired = x_derivative * 0
v_desired = np.sqrt(x_derivative ** 2 + y_derivative ** 2)
# plt.plot(np.linspace(0, 1, 60), v_desired)

# Calculate the desired angular velocity based on the angle's derivative
phi_derivative = np.gradient(phi_desired) / Ts
omega_desired = phi_derivative
# plt.plot(np.linspace(0, 1, 80), omega_desired)
# plt.show()
p2d = 0.0018 * 3  # converting pixel to real distance (m)


# p2d = 1/50.
# Function to find the control algorithm of the curved trajectory
def closed_loop_control(Kx, Ky, K_phi, x, y, phi, x_desired, y_desired, phi_desired, v_desired, omega_desired):
    v = Kx * (np.cos(phi) * (x_desired - x) * p2d + np.sin(phi) * (y_desired - y) * p2d) + v_desired * p2d * (
        np.cos((phi_desired - phi)))
    omega = K_phi * np.sin((phi_desired - phi)) + Ky * p2d * v_desired * (
            -np.sin(phi) * (x_desired - x) * p2d + np.cos(phi) * (y_desired - y) * p2d) + omega_desired

    return v, omega


# Function to calculate the velocities of both left and right wheels
def calc_wheels_velocities(v, omega, a):
    v_left = v / p2d + (32 * omega)  # to be consistent with img coordinates
    v_right = v / p2d - (32 * omega)
    # v_left = v - (a * omega)
    # v_right = v + (a * omega)
    return v_left, v_right


# Function to detect a specific color range and return coordinates
def detect_color_and_get_coordinates(frame, lower_blue_color, upper_blue_color, lower_red_color, upper_red_color):
    # Convert the frame to the HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create the mask for both colors
    # blue_mask = cv2.inRange(hsv_frame, lower_blue_color, upper_blue_color)
    # blue_color = cv2.bitwise_and(frame, frame, mask=blue_mask)

    coordinates = []

    red_mask1 = cv2.inRange(hsv_frame, lower_red_color, upper_red_color)

    lower_red_color_2 = np.array([345. // 2, 90, 160])
    upper_red_color_2 = np.array([360. // 2, 240, 255])
    red_mask2 = cv2.inRange(hsv_frame, lower_red_color_2, upper_red_color_2)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    # cv2.imshow("r",red_mask)
    # cv2.waitKey(5)
    M = cv2.moments(red_mask)

    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        coordinates.append((cX, cY))
    else:
        return 'nan'
    blue_mask = cv2.inRange(hsv_frame, lower_blue_color, upper_blue_color)
    M = cv2.moments(blue_mask)

    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        coordinates.append((cX, cY))
    else:
        return 'nan'
    # cv2.circle(blue_mask, (cX, cY), 10, (0, 255, 0), -1)
    # cv2.imshow("b", blue_mask)
    # cv2.waitKey(5)
    return coordinates  # coordinates of red part and blue part at the same time


# Function to calculate distance, angle, and midpoint of two coordinates
def calculate_distance_angle_and_midpoint(coord1, coord2):
    x1, y1 = coord1
    x2, y2 = coord2

    # Check if the coordinates are distinct
    if (x1, y1) != (x2, y2):
        # Calculate distance using the Euclidean distance formula
        distance = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        # Calculate angle using arctangent
        angle_rad = np.arctan2(y2 - y1, x2 - x1)
        angle_deg = np.degrees(angle_rad)

        # Calculate midpoint
        midpoint = ((x1 + x2) // 2, (y1 + y2) // 2)
        # print(x1, y1, x2, y2)
        return distance, angle_rad, midpoint
    else:
        # Coordinates are the same, return placeholder values
        return 'nan', 'nan', (0, 0)


# Blue color range
lower_blue_color = np.array([205 // 2, 150, 50])
upper_blue_color = np.array([230 // 2, 255, 220])

# Red color range
lower_red_color = np.array([0. // 2, 90, 180])
upper_red_color = np.array([15. // 2, 240, 255])
i = 0
coordinates = []
coordinates.append((0, 0))
coordinates.append((0, 0))
rightM = 0
leftM = 0
aux = 0

x = 0
y = 0

x_data = []
y_data = []
try:
    while True:
        start = time.time()
        # Load the frame
        ret, frame = cap.read()
        # print(np.shape(frame))
        draw_points = (np.asarray([np.floor(x_d), np.floor(y_d)]).T).astype(
            np.int32)  # needs to be int32 and transposed

        cv2.polylines(frame, [draw_points], False, (250, 250, 250), 5)
        # cv2.circle(frame, (x, y), 3, (0, 255, 0), -1)
        # cv2.circle(frame, (x + int(30*np.cos(phi)), y + int(30*np.sin(phi))), 3, (0, 255, 0), -1)
        # Detect the specified color in the frame and get coordinates
        coor = detect_color_and_get_coordinates(frame, lower_blue_color, upper_blue_color,
                                                lower_red_color, upper_red_color)
        if coor != 'nan':
            coordinates = coor

        # x = np.zeros((480, 640))
        # x[np.asinteger(np.floor(x_d)), np.asinteger(np.floor(y_d))] = 1
        # cv2.imshow("x",x)
        # Calculate the distance, angle, and midpoint between the two detected points
        distance, angle, mid_point = calculate_distance_angle_and_midpoint(coordinates[0], coordinates[1])
        if mid_point != 0:
            x, y = mid_point[0], mid_point[1]
            # print(mid_point)
        if angle != 'nan':
            phi = angle
            # print(angle)
        x_data.append(x)
        y_data.append(y)
        # print(x, y)
        # if x < 50 or y < 50:
        #     aux = 1
        # if aux == 0:
        #
        #     print(x, y)
        # else:
        #     print(x, y)

        #     break
        # Invoke the trajectory control algorithm
        v, omega = closed_loop_control(Kx, Ky, K_phi, x, y, phi, x_desired[i], y_desired[i], phi_desired[i],
                                       v_desired[i],
                                       omega_desired[i])

        if i < len(x_d) - 2:  # to continue until the end of path
            i = i + 1
        else:
            break
        # Invoke the robot kinematics
        v_left, v_right = calc_wheels_velocities(v, omega, a)
        v_right = v_right * 60 / 50.
        v_left = v_left * 60 / 50.

        # print(v_right, v_left)
        if v_right > 200:
            rightM = 200
        elif v_right < -200:
            rightM = -200
        else:
            rightM = int(np.floor(v_right))

        if v_left > 200:
            leftM = 200
        elif v_left < -200:
            leftM = -200
        else:
            leftM = int(np.floor(v_left))
        # rightM = 102
        # leftM = -100
        print((str(rightM) + str('/') + str(-leftM) + str('/')))
        bluetooth.write(
            (str(rightM) + str('/') + str(-leftM) + str('/')).encode())  # These need to be bytes not unicode

        # Display the original frame, with the path on it
        cv2.imshow("Original Frame", frame)

        cv2.waitKey(85)
        stop = time.time()
        print(start - stop)

except KeyboardInterrupt:
    print("Program is interrupted!")

bluetooth.write((str(0) + str('/') + str(0) + str('/')).encode())  # These need to be bytes not unicode
draw_points = (np.asarray([np.floor(x_d), np.floor(y_d)]).T).astype(np.int32)  # needs to be int32 and transposed

cv2.polylines(frame, [draw_points], False, (250, 250, 250), 5)

draw_points = (np.asarray([np.floor(x_data), np.floor(y_data)]).T).astype(np.int32)  # needs to be int32 and transposed

cv2.polylines(frame, [draw_points], False, (0, 250, 250), 5)
cv2.imshow("Result", frame)
cv2.waitKey()
# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()  # Converting the defined list into a NumPy array

plt.show()
