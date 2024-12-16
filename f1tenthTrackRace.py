#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import serial
import time

# Configure the serial port
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Adjust port if necessary
time.sleep(2)  # Wait for the connection to establish

# Initial threshold distances in meters
threshold_front = 1.0
threshold_side = 1.0

# Initialize the matplotlib figure
plt.ion()
fig, ax = plt.subplots()

# Detection angles in degrees
angles = {
    'left': -45,
    'front': 0,
    'right': 45
}

# Initial detection distances (arbitrary values)
detection_distances = {
    'left': 0.5,
    'front': 1.5,
    'right': 0.5
}

def send_parameters(speed, angle):
    """
    Send motor speed and servo angle to Arduino.
    :param speed: Motor speed (-255 to 255, negative for reverse).
    :param angle: Servo angle (0 to 180 degrees).
    """
    command = f"{speed},{angle}\n"  # Format: "SPEED,ANGLE\n"
    arduino.write(command.encode())  # Send command as bytes
    time.sleep(0.1)  # Small delay to ensure Arduino processes it

def adjust_thresholds(front, side):
    global threshold_front, threshold_side
    threshold_front = front
    threshold_side = side

def callback(data):
    global detection_distances
    
    # Extract the range data from the LaserScan message
    ranges = np.array(data.ranges)
    
    # Get the number of readings
    num_readings = len(ranges)
    
    # Calculate indices for the left, front, and right angles
    left_index = int((angles['left'] - data.angle_min) / data.angle_increment) % num_readings
    front_index = int((angles['front'] - data.angle_min) / data.angle_increment) % num_readings
    right_index = int((angles['right'] - data.angle_min) / data.angle_increment) % num_readings
    
    # Get distances for front, left, and right, ensuring valid values
    front_distance = ranges[front_index] if not np.isinf(ranges[front_index]) else float('nan')
    left_distance = ranges[left_index] if not np.isinf(ranges[left_index]) else float('nan')
    right_distance = ranges[right_index] if not np.isinf(ranges[right_index]) else float('nan')
    
    # Handle invalid values by setting a default large value (indicating no obstacle)
    front_distance = front_distance if not np.isnan(front_distance) else 10.0
    left_distance = left_distance if not np.isnan(left_distance) else 10.0
    right_distance = right_distance if not np.isnan(right_distance) else 10.0
    
    # Update detection distances
    detection_distances['front'] = front_distance
    detection_distances['left'] = left_distance
    detection_distances['right'] = right_distance

def update_plot():
    ax.clear()
    
    # Plot the robot's position
    ax.plot(0, 0, 'bo', label='Robot')
    
    # Convert angles and distances to Cartesian coordinates
    for key in angles:
        angle_rad = np.deg2rad(angles[key])
        distance = detection_distances[key]
        x = distance * np.cos(angle_rad)
        y = distance * np.sin(angle_rad)
        ax.plot([0, x], [0, y], label=f'{key.capitalize()} Detection')
    
    # Set plot limits
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    
    # Set labels and title
    ax.set_xlabel('X-axis (meters)')
    ax.set_ylabel('Y-axis (meters)')
    ax.set_title('LiDAR Detection Areas')
    ax.legend()
    
    # Draw the updated plot
    plt.draw()
    plt.pause(0.1)

def control_robot():
    # Initialize movement parameters
    global detection_distances
    
    speed = -37
    angle = 90  # Center
    
    # Decision making based on distances
    if detection_distances['front'] < threshold_front:
        speed = 0  # Stop if there's an obstacle in front
    else:
        if detection_distances['left'] < threshold_side:
            angle = 40  # Turn right if there's an obstacle on the left
        elif detection_distances['right'] < threshold_side:
            angle = 140  # Turn left if there's an obstacle on the right
    
    # Send parameters to Arduino
    send_parameters(speed, angle)

def listener():
    # Initialize the ROS node
    rospy.init_node('lidar_listener', anonymous=True)
    
    # Subscribe to the LiDAR topic
    rospy.Subscriber('/scan', LaserScan, callback)
    
    # Reduce sleep time to increase responsiveness
    while not rospy.is_shutdown():
        control_robot()
        update_plot()
        rospy.sleep(0.1)  # Decrease sleep time for faster loop

if __name__ == '__main__':
    # Adjust detection ranges if needed
    adjust_thresholds(front=1.0, side=0.4)
    listener()
    arduino.close()
    plt.show()

