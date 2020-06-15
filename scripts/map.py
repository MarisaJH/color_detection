#!/usr/bin/env python

'''
This is meant to be run on the local laptop instead of on the turtlebot.
'''
import subprocess
import sys
import os
import roslib; roslib.load_manifest('color_detection')
import rospy
import tf2_ros as tf
import tf2_geometry_msgs
import std_msgs.msg
from geometry_msgs.msg import Point, PointStamped
import yaml
import requests
import rospkg

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
    # with open("/home/marisahudspeth/catkin_ws/src/color_detection/zones/sketchedZone.yaml") as f:
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
    privacy_zones_path = "/home/" + os.getlogin() + "/Privacy-GUI/privacy_zones/sketchedPrivacyZones.py"

    # run map interface
    subprocess.call(["python", privacy_zones_path,
                     "{}/maps/cs_lounge.yaml".format(color_detection_path)],
                      stdin=subprocess.PIPE, stdout=subprocess.PIPE,  stderr=subprocess.STDOUT)

    # copy points from this computer to turtlebot
    cmd = "scp {}/zones/sketchedZone.yaml turtlebot@10.20.65.83:/home/turtlebot/catkin_ws/src/color_detection/zones/sketchedZone.yaml".format(color_detection_path)
    subprocess.call(cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE,  stderr=subprocess.STDOUT)

    # copy log of points from this computer to turtlebot
    cmd = "scp {}/log/sketchedLog.txt turtlebot@10.20.65.83:/home/turtlebot/catkin_ws/src/color_detection/log/sketchedLog.txt".format(color_detection_path)
    subprocess.call(cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE,  stderr=subprocess.STDOUT)

    '''
    subprocess.call(["python", "/home/marisahudspeth/catkin_ws/src/privacy_zones/sketchedPrivacyZones.py",
                     "/home/marisahudspeth/catkin_ws/src/color_detection/maps/cs_lounge.yaml"],
                      stdin=subprocess.PIPE, stdout=subprocess.PIPE,  stderr=subprocess.STDOUT)

    # copy points from this computer to turtlebot
    cmd = "scp /home/marisahudspeth/catkin_ws/src/color_detection/zones/sketchedZone.yaml turtlebot@10.20.65.83:/home/turtlebot/catkin_ws/src/color_detection/zones/sketchedZone.yaml"
    subprocess.call(cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE,  stderr=subprocess.STDOUT)

    # copy log of points from this computer to turtlebot
    cmd = "scp /home/marisahudspeth/catkin_ws/src/color_detection/log/sketchedLog.txt turtlebot@10.20.65.83:/home/turtlebot/catkin_ws/src/color_detection/log/sketchedLog.txt"
    subprocess.call(cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE,  stderr=subprocess.STDOUT)
    '''
    # only transform points from map to camera if we want
    # virtual feedback
    args = rospy.myargv(argv=sys.argv)
    print args
    if len(args) == 2 and args[1] == "toVirtual":
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
