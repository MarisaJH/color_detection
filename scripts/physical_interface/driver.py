#!/usr/bin/env python

# We always need these lines in a ROS python node.  They import all of
# the basic ROS functions, and make sure that the Python path is set
# up properly.  If you cut and paste these lines, make sure you change
# the manifest name to point to the one in the package that you're
# writing.  ROS will use whatever manifest you specify, even if it's
# not in the current package.  This can be *really* hard to debug.
import roslib; roslib.load_manifest('color_detection')
import rospy
import numpy
import os
import sys
import yaml
import rospkg
import tf2_ros as tf
import tf2_geometry_msgs
import std_msgs.msg
from geometry_msgs.msg import Point, PointStamped
from math import sqrt
import requests
from datetime import datetime
#import urllib
#import urllib2

# coor message with a bool for color detection and
# an array to store points msgs;
# points message that holds array of x,y coordinates
# for one segment (cone)
from color_detection.msg import coor, points

# service that converts image pixels of orange objects into 3d points
from color_detection.srv import pixelTo3dPoint

class driver:
    def __init__(self, feedback_type):
        # define what type of feedback will be provided: physical
        # or map (needs to write to a file), or virtual (needs to
        # upload data to server)
        self.feedback_type = feedback_type

        self.transform = -1
        self.tf_buffer = tf.Buffer()
        self.tf_listener = tf.TransformListener(self.tf_buffer)

        # subscribe to pixel coordinate data for orange cones
        self.coor_sub = rospy.Subscriber('color_detected', coor, self.driver_callback)

        self.pc2_srv = rospy.ServiceProxy('pixelTo3dPoint', pixelTo3dPoint)

        self.pkg_path = (rospkg.RosPack()).get_path('color_detection')


    def driver_callback(self, coor):
        # get transform from camera to map frame
        self.get_tf()

        # if orange cones were detected, send their
        # pixel coordinates to the pointcloud node
        print coor.colorDetected
        if coor.colorDetected and self.transform != -1:

            try:

                srv_res = self.pc2_srv(coor.segments)
                conePos = srv_res.conePos

                # this would mean the pc2_srv hasn't received pointcloud data yet,
                # or not enough cones were detected
                if conePos[0].x == -99 or len(conePos) < 4:
                    print(len(conePos))
                    return

                # map or virtual feedback
                if (self.feedback_type != "physical"):
                    # organize segments so there are only 4 (to make a rectangle)
                    if (len(conePos) > 4):
                        print("reducing segments")
                        conePos = self.reduce_segments(conePos)

                    # upload cone positions to server
                    if (self.feedback_type == "virtual"):
                        print("uploading positions")
                        self.upload_positions(conePos)

                # calculate convex hull
                print("calculting convex hull")
                hull = self.convex_hull(self.transform_points(conePos))

                # map or physical feedback
                if (self.feedback_type != "virtual"):
                    # write cone positions (hull) to file
                    zone_file = rospy.get_param('color_detection/zones', self.pkg_path + "/zones/physicalZone.yaml")
                    with open(zone_file, "w") as f:
                        zone = {"prohibition_areas": [hull]}
                        yaml.safe_dump(zone, f)

                    print 'Positions written to file'

                # log transformed positions
                self.log_positions(hull)


            except rospy.ServiceException, e:
                print "Service call failed: %s"%e


    def transform_points(self, conePos):
        tf_cone_pos = []
        for i in range(0,len(conePos)):

            # transform points from camera_rgb_optical_frame to map frame
            point_stamped = PointStamped()
            point_stamped.point = conePos[i]
            point_stamped.header = std_msgs.msg.Header()
            point_stamped.header.stamp = rospy.Time.now()
            point_stamped.header.frame_id = "camera_rgb_optical_frame"
            transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, self.transform)
            #print 'x,y,z', transformed_point.point.x,transformed_point.point.y, transformed_point.point.z
            tf_cone_pos.append([transformed_point.point.x, transformed_point.point.y])
        return tf_cone_pos

    def log_positions(self, positions):
        log_file = rospy.get_param('color_detection/zones', self.pkg_path + "/log/physicalLog.txt")

        with open(log_file, "a") as f:
            time = datetime.now().strftime("\n%d/%m/%Y %H:%M:%S\n")
            f.write(time)

            for pos in positions:
                str_pos = str(pos[0]) + " " + str(pos[1]) + "\n"
                f.write(str_pos)
        print "Positions logged"


    def get_tf(self):

        try:
            self.transform = self.tf_buffer.lookup_transform("map", "camera_rgb_optical_frame", rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print 'Waiting for transform...'


    # from https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain
    def convex_hull(self, points):
        """Computes the convex hull of a set of 2D points.

        Input: an iterable sequence of (x, y) pairs representing the points.
        Output: a list of vertices of the convex hull in counter-clockwise order,
        starting from the vertex with the lexicographically smallest coordinates.
        Implements Andrew's monotone chain algorithm. O(n log n) complexity.
        """

        # Sort the points lexicographically (tuples are compared lexicographically).
        # Remove duplicates to detect the case we have just one unique point.
        #points = sorted(set(points))
        points = sorted(points)


        # Boring case: no points or a single point, possibly repeated multiple times.
        if len(points) <= 1:
            return points

        # 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
        # Returns a positive value, if OAB makes a counter-clockwise turn,
        # negative for clockwise turn, and zero if the points are collinear.
        def cross(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

        # Build lower hull
        lower = []
        for p in points:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
                lower.pop()
            lower.append(p)

        # Build upper hull
        upper = []
        for p in reversed(points):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
                upper.pop()
            upper.append(p)

        # Concatenation of the lower and upper hulls gives the convex hull.
        # Last point of each list is omitted because it is repeated at the beginning of the other list.
        return lower[:-1] + upper[:-1]

    def reduce_segments(self, positions):
        #print "Reducing number of segments from", len(positions)
        while len(positions) != 4:
            # holds the distance for each point to its closest neighbor
            min_distances = [0]*len(positions)

            # find the minimum distances
            for i in range(len(positions)):
                min_distances[i] = sys.float_info.max
                for j in range(i+1, len(positions)):
                    pos1 = positions[i]
                    pos2 = positions[j]
                    dist = sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)
                    if dist < min_distances[i]:
                        min_distances[i] = dist

            # delete the segment with the smallest distance
            min_index = min_distances.index(min(min_distances))
            del min_distances[min_index]
            del positions[min_index]

        return positions

    def upload_positions(self, positions):
        # upload to server
        final_positions = ""
        for pos in positions:
            final_positions += str(pos.x) + " " + str(pos.y) + " " + str(pos.z) + "\n"

        r = requests.put("http://cs.rhodes.edu/hudspethm/virtualFeedbackZone.yaml", data=final_positions)
        #print r.status_code

        print "Positions uploaded"
        #for pos in positions:
        #    print pos, "\n"




if __name__ == "__main__":
    #Initialize driver node
    rospy.init_node('driver', anonymous=True, log_level=rospy.DEBUG)

    args = rospy.myargv(argv=sys.argv)
    feedback_type =args[1]

    driver = driver(feedback_type)

    rospy.spin()
