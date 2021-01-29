#!/usr/bin/env python

'''
This program should be run on the local laptop, not the turtlebot. This runs the map interface on the
local laptop since it is easier to see and use that way (instead of having it run on the turtlebot).

This program helps automate some of the process of going from the map interface to either the physical
or virtual feedback.
'''
import subprocess
import sys
from os import getlogin
import roslib; roslib.load_manifest('color_detection')
import rospy
import tf2_ros as tf
import tf2_geometry_msgs
import std_msgs.msg
from geometry_msgs.msg import Point, PointStamped
import yaml
import requests
import rospkg
import argparse

def get_tf(tf_buffer):
    transform = -1
    while transform == -1:
        try:
            transform = tf_buffer.lookup_transform("camera_rgb_optical_frame", "map", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "Waiting for transform..."
            continue

    return transform

def read_points():
    with open("{}/zones/sketchedZone.yaml".format(color_detection_path)) as f:
        zone = yaml.safe_load(f)
        zone = zone['prohibition_areas'][0]
    zone_list = []
    for z in zone:
        point = Point()
        point.x = float(z[0])
        point.y = float(z[1])
        point.z = float(z[2]) 

        zone_list.append(point)
    return zone_list

def transform_points(transform, points):
    tf_points = ""
    for point in points:
        point_stamped = PointStamped()
        point_stamped.point = point
        point_stamped.header = std_msgs.msg.Header()
        point_stamped.header.stamp = rospy.Time.now()
        point_stamped.header.frame_id = "map"

        transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        tf_points += str(transformed_point.point.x) + " " + str(transformed_point.point.y) + " " + str(transformed_point.point.z) + "\n"
    print tf_points
    return tf_points

if __name__ == "__main__":
    color_detection_path = (rospkg.RosPack()).get_path('color_detection')
    privacy_zones_path = "/home/" + getlogin() + "/Privacy-GUI/privacy_zones/sketchedPrivacyZones.py"
    tbot_color_detection_path = "/home/turtlebot/catkin_ws/src/color_detection"
    remote_host =  "turtlebot@192.168.1.204" #"turtlebot@10.20.65.83"

    # set up arguments: feedback_type is required, others are optional 
    parser = argparse.ArgumentParser(description="Automate the process of going from the map interface to either physical or virtual feedback.")
    parser.add_argument("feedback_type", type=str, help="The feedback type you want to see. Either physical or virtual")
    parser.add_argument("-m", "--map", 
                        default="{}/maps/cs_lounge.yaml".format(color_detection_path), 
                        help="Path to your map file on the local laptop.")
    parser.add_argument("-t", "--tbot_path", 
                        default=tbot_color_detection_path, 
                        help="Path to the color_detection package on the turtlebot")
    parser.add_argument("-r", "--remote_host", 
                        default=remote_host, 
                        help="username@host (the turtlebot)")

    args = parser.parse_args()

    # run map interface
    subprocess.call(["python", privacy_zones_path, args.map], 
                      stdin=subprocess.PIPE, stdout=subprocess.PIPE,  stderr=subprocess.STDOUT)

    # copy points from this computer to turtlebot
    cmd = "scp {}/zones/sketchedZone.yaml {}:{}/zones/sketchedZone.yaml".format(color_detection_path, args.remote_host, args.tbot_path)
    subprocess.call(cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE,  stderr=subprocess.STDOUT)

    # copy log of points from this computer to turtlebot
    cmd = "scp {}/log/sketchedLog.txt {}:{}/log/sketchedLog.txt".format(color_detection_path, args.remote_host, args.tbot_path)
    subprocess.call(cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE,  stderr=subprocess.STDOUT)
    
    # only transform points from map to camera if we want 
    # virtual feedback
    #args = rospy.myargv(argv=sys.argv)
    #print args
    if args.feedback_type == "virtual":
        rospy.init_node('sketched_to_virtual', anonymous=True, disable_signals=True)
        print "Initialized node"
        
        tf_buffer = tf.Buffer()
        tf_listener = tf.TransformListener(tf_buffer)
        transform = get_tf(tf_buffer)
        print "Got transform"
        points = read_points()
        print "Read points"
        tf_points = transform_points(transform, points)
        print "Transformed points"
     
        r = requests.put("http://cs.rhodes.edu/hudspethm/virtualFeedbackZone.yaml", data=tf_points)
        #print r.status_code
        rospy.signal_shutdown("Cone positions successfully transformed and uploaded")



