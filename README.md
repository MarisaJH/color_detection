# color_detection

This package allows you to create a privacy zone (or "no-go" region) that a turtlebot won't drive through. 

There are three methods available to create the privacy zone:
1. **Physical**: Four real, orange cones are placed on the ground to mark the corners of the no-go region.
2. **Map/Sketched**: Four points are clicked on a map of the room to mark the corners of the no-go region.
3. **AR/Virtual**: Four virtual cones are placed on the ground in an app by tapping the screen.

There are three feedback types to see if the privacy zone works:
1. **Physical**: The robot drives around the room and avoids the no-go region.
2. **Map/Sketched**: The no-go region is shaded red on a map of the room.
3. **AR/Virtual**: A virtual fence appears on the screen where the no-go region was specified. 

