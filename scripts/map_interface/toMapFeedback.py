#!/usr/bin/env python

import yaml
import sys
import rospkg
from cv2 import imread

# Get the origin and resolution from the slam map yaml file
def yaml_to_meta_data(file_name):
	yamlInfo = yaml.load(open(file_name))
	res = yamlInfo["resolution"]
	origin = yamlInfo["origin"]
	img = yamlInfo["image"]
	return (res, origin, img)

if __name__ == "__main__":
    # TODO CHANGE TO TURTLEBOT
    map_file = "/home/marisahudspeth/catkin_ws/src/color_detection/maps/cs_lounge.yaml"
    color_detection_path = (rospkg.RosPack()).get_path('color_detection')
    interface_file = color_detection_path


    if len(sys.argv) < 2:
        print("Program requires at least one argument (interface type)")
        sys.exit(2)
    elif len(sys.argv) > 1:
        interface_type = sys.argv[1]
        if interface_type == "virtual":
            interface_file += "/zones/virtualZone.yaml"
        elif interface_type == "physical":
            interface_file += "/zones/zone.yaml"
        else:
            print("Invalid interface type given. Choices are physical or virtual")
            sys.exit(2)
    if len(sys.argv) == 3:
        map_file = sys.argv[2]

    print(interface_type)
    print(map_file)
    
    res, origin, img = yaml_to_meta_data(map_file)
    dst = imread(img, 0)
    img_height, img_width = dst.shape

    # get slam map coordinates for private zone
    with open(interface_file, "r") as f:
        interface_yaml = yaml.safe_load(f)

        points = interface_yaml["prohibition_areas"][0]

    
    # transform slam map points to pixel/image
    new_points = []
    for point in points:
        x = (point[0] - origin[0]) / res
        y = ((origin[1] - point[1]) / res) + img_height

        new_points.append([x, y])
    
    # write new points to file
    zone_file = color_detection_path + "/zones/{}ToMap.yaml".format(interface_type)
    with open(zone_file) as f:
        map_yaml = yaml.safe_load(f)
    
    i = 0
    for point in map_yaml["Zone List"][0]["Points"]:
        point["x"] = new_points[i][0]
        point["y"] = new_points[i][1]
        i += 1

    with open(zone_file, "w") as f:
        yaml.safe_dump(map_yaml, f)
