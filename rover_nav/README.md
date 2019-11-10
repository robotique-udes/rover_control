# rover_nav dependencies
* mapviz
* docker
* robot_localization 2.6.5

# Mapviz

### Dependencies
* mapviz

### Installing mapviz
https://github.com/swri-robotics/mapviz

### Launching mapviz only
roslaunch mapviz mapviz.launch

### Launching mapviz + simulated GPS values
roslaunch rover_nav simulation.launch

### Displaying gps coordinated on map
1. Add gps plugin (Add -> gps -> Ok)
2. Select topic "/pos" (First point is at coordinates 45.5,-73.5)
3. Add tile_map plugin (Add -> tile_map -> Ok)

# How to get Google Maps satelite imagery in Mapviz
### Dependencies
* mapviz
* docker

**Note:** An internet connection is needed the first time, however, the tile map is cached so it can then be used offline.

### Installing Docker
Follow this tutorial, starting at "Install using the repository"
https://docs.docker.com/v17.12/install/linux/docker-ce/ubuntu/#uninstall-old-versions

### Setting up and running mapproxy server:
Follow instructions of section 1 only.
https://github.com/danielsnider/MapViz-Tile-Map-Google-Maps-Satellite

### To do at every boot (<em>TODO: automate this process on startup</em>):
* `sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy`
* `Base url for the tile map plugin (if not there already): http://localhost:8080/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png`

# Waypoints

### Listing waypoint coordinates
Coordinates are listed in `gpsGoals.txt`. Each point must be written on a seperate line. You can write comments on a line or at the end
of a coordinate by prefacing it with a `#`. Comments will be ignored by the parser.

### Displaying ideal path in Mapviz
1. Run mapviz and add the path plugin (Add -> path -> Ok)
2. Run the parsePathServer with `rosrun rover_nav parsePathServer.py`
3. Change the content of gpsGoals.txt to the desired coordinates and save.
4. In the path plugin in mapviz, choose the topic "/path_topic"
5. Call the service with `rosservice call /parsePath`
6. The path should now display on the map. Recall the service anytime you modify gpsGoals.txt to apply the changes.

### Setting waypoint as a goal
** The path must be published before doing this part, see previous section **
1. Make sure parsePathServer is running and that the path is published
2. Call the service with `rosservice call /setWaypoint <waypointNumber>`

# Installing robot_localization
1. Download the source code https://github.com/cra-ros-pkg/robot_localization/releases/tag/2.6.5
2. Extract to /{workspace}/src/
3. run `rospack profile`in a terminal to to force an update of the package cache
4. build by going to the root of the worspace and using the command `catkin_make`
