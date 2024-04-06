import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# Open the camera (use 0 for the default camera)
cap = cv2.VideoCapture(0)

ret, frame = cap.read()


def draw_curve_with_mouse_events(event, x, y, flags, params):
    global lines_arr, stop
    if event == cv2.EVENT_LBUTTONDOWN and stop == 0:
        print(x, y)
        lines_arr.append((x, y))

    elif event == cv2.EVENT_LBUTTONUP and stop == 0:
        lines_arr.append((x, y))
        stop = 1

    elif event == cv2.EVENT_MOUSEMOVE and (flags and cv2.EVENT_FLAG_LBUTTON) and stop == 0:
        lines_arr.append((x, y))


# Create a window
cv2.namedWindow("Original Frame")

# Set the callback function
cv2.setMouseCallback("Original Frame", draw_curve_with_mouse_events)

lines_arr = []
stop = 0

ret, frame = cap.read()
while stop == 0:
    cv2.imshow("Original Frame", frame)
    cv2.waitKey(5)
    for line_start_point in range(1, len(lines_arr)):
        cv2.line(frame, lines_arr[line_start_point - 1], lines_arr[line_start_point], (0, 0, 0), 6)

print(lines_arr)


# Function to detect a specific color range and return coordinates
def detect_color_and_get_coordinates(frame, lower_blue_color, upper_blue_color, lower_red_color, upper_red_color):
    # Convert the frame to the HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create the mask for both colors
    # blue_mask = cv2.inRange(hsv_frame, lower_blue_color, upper_blue_color)
    # blue_color = cv2.bitwise_and(frame, frame, mask=blue_mask)

    red_mask = cv2.inRange(hsv_frame, lower_red_color, upper_red_color)
    red_color = cv2.bitwise_and(frame, frame, mask=red_mask)

    # Combine two masks to get both color regions
    # combined_mask = cv2.bitwise_or(blue_mask, red_mask)
    # combined_color = cv2.bitwise_and(frame, frame, mask=combined_mask)

    # Find contours in the mask
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Initialize a list to store the coordinates
    coordinates = []

    # Iterate through the detected contours and get their coordinates
    for contour in contours:
        M = cv2.moments(contour)

        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            coordinates.append((cX, cY))

    # Draw the contours on the frame
    cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)

    return red_color, coordinates


# Function to calculate distance, angle, and midpoint of two coordinates
def calculate_distance_angle_and_midpoint(coord1, coord2):
    x1, y1 = coord1
    x2, y2 = coord2

    # Check if the coordinates are distinct
    if (x1, y1) != (x2, y2):
        # Calculate distance using the Euclidean distance formula
        distance = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        # Calculate angle using arc tangent
        angle_rad = np.arctan2(y2 - y1, x2 - x1)
        angle_deg = np.degrees(angle_rad)

        # Calculate midpoint
        midpoint = ((x1 + x2) // 2, (y1 + y2) // 2)

        return distance, angle_deg, midpoint
    else:
        # Coordinates are the same, return placeholder values
        return float('nan'), float('nan'), (0, 0)


# Blue color range
lower_blue_color = np.array([101, 50, 38])
upper_blue_color = np.array([110, 255, 255])

# Red color range
lower_red_color = np.array([160, 20, 70])
upper_red_color = np.array([190, 255, 255])

# Declare control algorithm parameters
Kx = 1
Ky = 1
K_phi = 1
a = 11.5 / 2

try:
    while True:
        # Load the frame
        ret, frame = cap.read()

        # Draw the path
        for line_start_point in range(1, len(lines_arr)):
            cv2.line(frame, lines_arr[line_start_point - 1], lines_arr[line_start_point], (0, 0, 0), 6)

        # Detect the specified color in the frame and get coordinates
        detected_frame, coordinates = detect_color_and_get_coordinates(frame, lower_blue_color, upper_blue_color,
                                                                       lower_red_color, upper_red_color)

        if len(coordinates) >= 2:
            # Calculate the distance, angle, and midpoint between the two detected points
            distance, angle, mid_point = calculate_distance_angle_and_midpoint(coordinates[0], coordinates[1])

            # Draw a line between the two detected points
            cv2.line(frame, coordinates[0], coordinates[1], (0, 255, 0), 2)

            # Draw a circle at the midpoint
            cv2.circle(frame, mid_point, 5, (0, 255, 0), -1)

            print("Distance:", distance)
            print("Angle:", angle)
            print("Midpoint Coordinates:", mid_point)

            # Perform the trajectory control algorithm
            v, omega = closed_loop_control(Kx, Ky, K_phi, x, y, phi, x_desired, y_desired, phi_desired, v_desired,
                                           omega_desired)

            # Calculate the robots' wheels
            v_left, v_right = calc_wheels_velocities(v, omega, a)

            print("Linear Velocity:", v)
            print("Angular Velocity:", omega)
            print("Velocity for the Left Wheel:", v_left)
            print("Velocity for the Right Wheel:", v_right)

        # Display the original frame, the detected frame with contours, and the coordinates
        cv2.imshow("Original Frame", frame)
        cv2.imshow("Detected Frame", detected_frame)
        print("Coordinates:", coordinates)

        key = cv2.waitKey(1)
        if key == 27:
            break
        elif key == ord("D"):  # Deleting circles
            lines_arr = []
except KeyboardInterrupt:
    print("Program is interrupted!")

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()


# Converting the defined list into a NumPy array
lines_arr_np = np.array(lines_arr)

# Identify x and y data points
x_desired = lines_arr_np[:, 0]
y_desired = lines_arr_np[:, 1]

# Calculate the desired angle of the desired trajectory
phi = 0
phi_desired[:-1] = np.arctan2(x_desired[1:] - x_desired[:-1], y_desired[1:] - y_desired[:-1])


# Declare curve fitting function
def fit_curve(x, a, b, c):
    return a * ((x + b) ** 2) + c

# Plot determined x and y data points
plt.plot(x_desired, y_desired, 'bo')

# Implement curve fitting
popt, pcov = curve_fit(fit_curve, x_desired, y_desired)
print(popt)

# Display the shape of data points
print(lines_arr_np.shape)

# Display the plot
plt.scatter(x_desired, y_desired)
plt.show()

# Plot the fitted curve function
plt.plot(x_desired, fit_curve(x_desired, *popt))

# Calculate the desired velocity based on the derivatives of x and y coordinated values
x_derivative = np.gradient(x_desired)
y_derivative = np.gradient(y_desired)
v_desired[:-1] = np.linalg.norm([x_derivative[:-1], y_derivative[:-1]], axis=0)

# Calculate the desired angular velocity based on the angle's derivative
phi_derivative = np.gradient(phi_desired)
omega_desired = phi_derivative
