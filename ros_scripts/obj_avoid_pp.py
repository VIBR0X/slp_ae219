#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from iq_gnc.py_gnc_functions import *
from math import cos, sin, pow, radians, sqrt
from geometry_msgs.msg import Point

# Initialize the drone control API as a global variable.
drone = gnc_api()

def laser_cb(msg):
    """Callback function for LaserScan messages."""
    avoid_x, avoid_y = 0.0, 0.0
    avoid = False
    d0 = 3.0  # Safety distance
    k = 0.5   # Repulsion coefficient

    # Process each range in the LaserScan
    for i in range(len(msg.ranges)):
        range_i = msg.ranges[i]
        if 0.35 < range_i < d0:
            avoid = True
            angle = msg.angle_increment * i
            x = cos(angle)
            y = sin(angle)
            u = -0.5 * k * pow(((1/range_i) - (1/d0)), 2)
            avoid_x += x * u
            avoid_y += y * u

    # Rotate the avoidance vector by the current heading of the drone
    cr_heading = radians(drone.get_current_heading())
    global_x = avoid_x * cos(cr_heading) - avoid_y * sin(cr_heading)
    global_y = avoid_x * sin(cr_heading) + avoid_y * cos(cr_heading)

    if avoid:
        dist = sqrt(global_x**2 + global_y**2)
        max_dist = 3.0  # Maximum distance the drone can be repelled
        if dist > max_dist:
            global_x = max_dist * (global_x / dist)
            global_y = max_dist * (global_y / dist)

        cur_pose = drone.get_current_location()
        # Move the drone to the new safe position
        drone.set_destination(global_x + cur_pose.x, global_y + cur_pose.y, 2, 0)
    else:
        # No obstacles detected, proceed to next waypoint
        drone.set_destination(10, 0, 2, 0)


def main():
    rospy.init_node("obstacle_avoidance_node", anonymous=True)
    rospy.Subscriber("/spur/laser/scan", LaserScan, laser_cb, queue_size=1)

    # Setup and takeoff sequence
    drone.wait4connect()
    drone.wait4start()
    drone.initialize_local_frame()
    drone.takeoff(2)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting down")
        exit()