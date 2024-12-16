#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np

# Initialize the matplotlib figure
plt.ion()
fig, ax = plt.subplots()

def callback(data):
    # Extract the range data from the LaserScan message
    ranges = np.array(data.ranges)
    
    # Create an angle array for plotting
    angles = np.linspace(data.angle_min, data.angle_max, len(ranges))
    
    # Convert to Cartesian coordinates for plotting
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)
    
    # Clear the previous plot
    ax.clear()
    
    # Plot the LiDAR data
    ax.plot(x, y, 'ro')
    
    # Set plot limits
    ax.set_xlim([-10, 10])
    ax.set_ylim([-10, 10])
    
    # Draw the updated plot
    plt.draw()
    plt.pause(0.01)

def listener():
    # Initialize the ROS node
    rospy.init_node('lidar_listener', anonymous=True)
    
    # Subscribe to the LiDAR topic
    rospy.Subscriber('/scan', LaserScan, callback)
    
    # Keep the script running
    rospy.spin()

if __name__ == '__main__':
    listener()
    plt.show()
