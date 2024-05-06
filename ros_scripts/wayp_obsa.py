import rospy
from sensor_msgs.msg import LaserScan
from iq_gnc.py_gnc_functions import *
from math import atan2, cos, sin, pow, radians, sqrt
from geometry_msgs.msg import Point

drone = gnc_api()


def laser_cb(msg):
    cr_scan = LaserScan()
    cr_scan = msg
    avoid_x = 0.0
    avoid_y = 0.0
    avoid = False

    for i in range(1, len(cr_scan.ranges)):
        d0 = 3.0
        k = 0.5
        if cr_scan.ranges[i] < d0 and cr_scan.ranges[i] > 0.35:
            avoid = True
            x = cos(cr_scan.angle_increment * i)
            y = sin(cr_scan.angle_increment * i)
            u = (-0.5 * k * pow(((1/cr_scan.ranges[i]) - (1/d0)), 2.0))

            avoid_x += (x*u)
            avoid_y += (y*u)

    cr_heading = radians(drone.get_current_heading())
    avoid_x = (avoid_x * cos(cr_heading)) - (avoid_y * sin(cr_heading))
    avoid_y = (avoid_x * sin(cr_heading)) + (avoid_y * cos(cr_heading))

    if avoid:
        dist = sqrt(pow(avoid_x, 2) + pow(avoid_y, 2))

        if dist > 3:
            avoid_x = (3 * (avoid_x/dist))
            avoid_y = (3 * (avoid_y/dist))

        cur_pose = Point()
        cur_pose = drone.get_current_location()
        drone.set_destination(avoid_x + cur_pose.x,
                              avoid_y + cur_pose.y,
                              2, 0)


def main():
    rospy.init_node("obs_avoider", anonymous=True)
    rospy.Subscriber(name="/spur/laser/scan",
                     data_class=LaserScan,
                     queue_size=1,
                     callback=laser_cb)

    drone.wait4connect()
    drone.wait4start()
    drone.initialize_local_frame()
    drone.takeoff(2)

    # Set start and end waypoints
    start_x = 0.0
    start_y = 0.0
    end_x = 10.0
    end_y = 0.0

    cr_scan = LaserScan()

    # Calculate distance and angle to end waypoint
    cur_pose = drone.get_current_location()
    dist_to_end = sqrt(pow(end_x - cur_pose.x, 2) + pow(end_y - cur_pose.y, 2))
    angle_to_end = atan2(end_y - cur_pose.y, end_x - cur_pose.x)

    # Fly to end waypoint while avoiding obstacles
    while dist_to_end > 0.5:
        # Check for obstacles
        avoid_x = 0.0
        avoid_y = 0.0
        avoid = False
        for i in range(1, len(cr_scan.ranges)):
            d0 = 3.0
            k = 0.5
            if cr_scan.ranges[i] < d0 and cr_scan.ranges[i] > 0.35:
                avoid = True
                x = cos(cr_scan.angle_increment * i)
                y = sin(cr_scan.angle_increment * i)
                u = (-0.5 * k * pow(((1/cr_scan.ranges[i]) - (1/d0)), 2.0))
                avoid_x += (x*u)
                avoid_y += (y*u)
        
        # Transform avoid vector to drone frame
        cr_heading = radians(drone.get_current_heading())
        avoid_x = (avoid_x * cos(cr_heading)) - (avoid_y * sin(cr_heading))
        avoid_y = (avoid_x * sin(cr_heading)) + (avoid_y * cos(cr_heading))
        
        # Adjust avoid vector based on distance to obstacle
        if avoid:
            dist_to_obstacle = sqrt(pow(avoid_x, 2) + pow(avoid_y, 2))
            if dist_to_obstacle > 3:
                avoid_x = (3 * (avoid_x/dist_to_obstacle))
                avoid_y = (3 * (avoid_y/dist_to_obstacle))
        
        # Calculate new destination based on avoid vector
        new_dest_x = avoid_x + cur_pose.x
        new_dest_y = avoid_y + cur_pose.y
        
        # Fly to new destination
        drone.set_destination(new_dest_x, new_dest_y, 2, angle_to_end)
        
        # Update current pose and distance to end waypoint
        cur_pose = drone.get_current_location()
        dist_to_end = sqrt(pow(end_x - cur_pose.x, 2) + pow(end_y - cur_pose.y, 2))

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
       