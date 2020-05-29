# color_detection

This package is part of a larger project that allows you to create a privacy zone (or "no-go" region) that a turtlebot won't drive through. 

There are three methods available to create the privacy zone:
1. **Physical**: Four real, orange cones are placed on the ground to mark the corners of the no-go region.
2. **Map**: Four points are clicked on a map of the room to mark the corners of the no-go region.
3. **AR**: Four virtual cones are placed on the ground in an app by tapping the screen.

There are three feedback types to see if the privacy zone works:
1. **Physical**: The robot drives around the room and avoids the no-go region.
2. **Map**: The no-go region is shaded red on a map of the room.
3. **AR**: A virtual fence appears on the screen where the no-go region was specified. 

This package contains code for using the *physical* method with either *physical* or *AR* feedback.

## Guide
### Requirements
* Opencv
* [Costmap Prohibition Layer](http://wiki.ros.org/costmap_prohibition_layer)
* [Tf2](http://wiki.ros.org/tf2)
* Ros Kinetic

### Setup
* Optional: Create a SLAM map of the room. If you don't have one, a default map will be used.
* To make the Costmap Prohibition Layer work, add the following line on the turtlebot to opt/ros/kinetic/share/turtlebot_navigation/param/global_costmap_params.yaml under Plugins and before the inflation layer:
```
- {name: costmap_prohibition_layer,       type: "costmap_prohibition_layer_namespace::CostmapProhibitionLayer"}
```

### How to Run
1. Place four cones on the floor where you want to make a private area
2. Position the robot so that it can see all the cones
3. Open a terminal on the turtlebot
4. Type **python amcl_rviz.py**
5. Set the robot’s position to where it currently is in the room by clicking “2d pose estimate.” Then click and hold the mouse on the part of the map that the robot occupies in real life. While holding the mouse, orient the robot so that it is facing the same direction that it is in real life. Then release the mouse.
6. If you want to see physical feedback (ie, the robot drives around the private area), do the following:  
    7. Open another terminal and type **roslaunch color_detection color_detection.launch**  
    8. After the program says “Writing zones to file” press Ctrl + C to quit the program  
    9. Close out of rviz and amcl by typing Ctrl + C   
    10. Now type **python amcl_rviz.py physical**  
    11. Now you can see on the map the zone that you specified as private  
    12. If the robot is told to drive anywhere in the room, it won’t be able to go in that zone  
7. If you want to see virtual feedback (ie, see virtual walls that mark the private zone on a phone), do the following:  
    8. Open a terminal and type **roslaunch color_detection color_detection.launch feedback_type:=virtual**  
    9. Exit the program once it has printed the positions.  
    10. NOTE: the feedback type can be set to anything other than “physical” in order for this to work  
    11. Close out of rviz and amcl by typing Ctrl + C   
    12. On the phone, stand in the same place the robot is standing (you can hold the phone over the robot) and open the black app.   
    13. In the menu, click the “Other to Virtual Feedback” button  
