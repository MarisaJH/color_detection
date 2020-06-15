#!/usr/bin/env python

# We always need these lines in a ROS python node.  They import all of
# the basic ROS functions, and make sure that the Python path is set
# up properly.  If you cut and paste these lines, make sure you change
# the manifest name to point to the one in the package that you're
# writing.  ROS will use whatever manifest you specify, even if it's
# not in the current package.  This can be *really* hard to debug.
import roslib; roslib.load_manifest('color_detection')
import rospy
import os
# os.system(command)
import yaml
import urllib2
import tf2_ros as tf
import tf2_geometry_msgs
import std_msgs.msg
import rospkg
from geometry_msgs.msg import Point, PointStamped
from datetime import datetime

def get_tf(tf_buffer):

    try:
        transform = tf_buffer.lookup_transform("map", "camera_rgb_optical_frame", rospy.Time(0))
            
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        transform = -1
        print 'Waiting for transform...'
    
    return transform

def transform_points(zone_points, transform):
    transformed_points = []
    for point in zone_points:
        point_stamped = PointStamped()
        point_stamped.point = point
        point_stamped.header = std_msgs.msg.Header()
        point_stamped.header.stamp = rospy.Time.now()
        point_stamped.header.frame_id = "camera_rgb_optical_frame"

        transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        transformed_points.append([transformed_point.point.x, transformed_point.point.y])

    # write cone positions to file
    pkg_path = (rospkg.RosPack()).get_path('color_detection')
    zone_file = rospy.get_param('color_detection/zones', pkg_path + "/zones/virtualZone.yaml")
    with open(zone_file, "w") as f:
        zone = {"prohibition_areas": [transformed_points]}
        
        yaml.safe_dump(zone, f)
        f.close()
    print 'Zones written to file'
    
    # log positions with date and time
    log_file = rospy.get_param('color_detection/zones', pkg_path + "/log/virtualLog.txt")
    with open(log_file, "a") as f:
        time = datetime.now().strftime("\n%d/%m/%Y %H:%M:%S\n")
        f.write(time)

        for pos in transformed_points:
            str_pos = str(pos[0]) + " " + str(pos[1]) + "\n"
            f.write(str_pos)
    print "Positions logged"

if __name__ == "__main__":  
    rospy.init_node('cone_transformer', anonymous=True, log_level=rospy.DEBUG, disable_signals=True)

    zone_points = []
    fhand = urllib2.urlopen('http://cs.rhodes.edu/~hudspethm/zone.yaml')
    for line in fhand:
        #print(line.decode().strip())
        x, y, z = line.split()
        x = float(x)
        y = float(y)
        z = float(z)
        print x, y, z
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        zone_points.append(point)

    tf_buffer = tf.Buffer()
    tf_listener = tf.TransformListener(tf_buffer)

    while not rospy.is_shutdown():
        #transform = get_tf(tf_buffer)
        try:
            transform = tf_buffer.lookup_transform("map", "camera_rgb_optical_frame", rospy.Time(0))
            print 'Got transform'
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            transform = -1
            print 'Waiting for transform...'

        if transform == -1:
            continue

        transform_points(zone_points, transform)
        rospy.signal_shutdown("Cone positions successfully transformed")
    #rospy.spin()