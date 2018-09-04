# Pathfinder
A library for generating a set of navigation waypoints.

#### Uses
Pathfinder takes a 2d array or a bitmap image, and create a path to navigate from point *a* to point *b*. It does this by returning an array of pixel locations of every point to get to point *a* to point *b* smoothly.
It can also reduce this path to a path of straight lines.

##### FRC Specific
Pathfinder also has FRC specific implementation for navigating this path. It takes a current location (x, y) and rotation of the robot, and gives back the power for the left and right motors.

#### Creating a MotorOutput for use primarly in FRC.
```Java
WidthImageNodeMap imgMap = new WidthImageNodeMap("images/Picture1.png");
imgMap.setSafeWidth(40);
imgMap.setActualWidth(24);

FRCPathFinder pathfinder = new FRCPathFinder();
pathfinder.setNodeMap(imgMap);

FRCResult result = pathfinder.aStarSearch(new Node(162, 50), new Node(162, 230));

Waypoint[] nodes = result.pathToWaypoints(0);

MotorOutput mtr = new MotorOutput(12, nodes);
```
