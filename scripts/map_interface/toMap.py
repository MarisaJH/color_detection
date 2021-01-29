#!/usr/bin/env python
'''
This program should be run on the local laptop, not the turtlebot. This runs the map interface on the
local laptop since it is easier to see and use that way (instead of having it run on the turtlebot).

This program automates some of the process of going from either physical or virtual interfaces to
map feedback.
'''
import subprocess
import sys
from os import getlogin
import rospkg
import argparse

if __name__ == "__main__":
    # default values
    color_detection_path = (rospkg.RosPack()).get_path('color_detection')
    tbot_color_detection_path = "/home/turtlebot/catkin_ws/src/color_detection"
    privacy_zones_path = "/home/" + getlogin() + "/Privacy-GUI/privacy_zones/sketchedPrivacyZones.py"
    #map_file = "/home/marisahudspeth/catkin_ws/src/color_detection/maps/cs_lounge.yaml"
    remote_host =  "turtlebot@192.168.1.204" #"turtlebot@10.20.65.83"

    
    # set up arguments: interface_type is required, others are optional 
    parser = argparse.ArgumentParser(description="Automate the process of going from an interface to map feedback")
    parser.add_argument("interface_type", type=str, help="The interface used to create the privacy zone. Either physical or virtual.")
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
    
    # copy points from turtlebot to this computer
    cmd = "scp {}:{}/zones/{}Zone.yaml {}/zones/{}Zone.yaml".format(args.remote_host, args.tbot_path, 
                                                                    args.interface_type, color_detection_path, 
                                                                    args.interface_type)
    subprocess.call(cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE,  stderr=subprocess.STDOUT)

    # convert slam map points to pixel locations
    cmd = "python toMapFeedback.py {} {}".format(args.interface_type, args.map)
    subprocess.call(cmd, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE,  stderr=subprocess.STDOUT)

    # run map module to get map feedback
    subprocess.call(["python", privacy_zones_path, args.map], 
                      stdin=subprocess.PIPE, stdout=subprocess.PIPE,  stderr=subprocess.STDOUT)