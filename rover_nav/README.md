# rover_nav dependencies
* mapviz
* docker
* geographiclib
* robot_localization 2.6.5

# Mapviz

### Dependencies
* mapviz

### Installing mapviz
https://github.com/swri-robotics/mapviz

### Launching mapviz only
roslaunch mapviz mapviz.launch

### Configuring Mapviz
After opening Mapviz, click File->Open config and open /path/to/rover_nav/mapvizconfig.mvc
**Remember to reopen the config files if it was modified**

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

### Automate startup of the mapproxy server at boot:
1. Open a terminal and type `sudo crontab -e`
2. Choose any text editor
3. Add this line after the comments: `@reboot sudo docker run -p 8080:8080 -d -t -v ~/mapproxy:/mapproxy danielsnider/mapproxy`

**Base url for the tile map plugin (if not there already):** `http://localhost:8080/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png`

# Waypoints

### Listing waypoint coordinates
Coordinates are listed in `gpsGoals.txt`. Each point must be written on a seperate line. You can write comments on a line or at the end
of a coordinate by prefacing it with a `#`. Comments will be ignored by the parser.

### Displaying ideal path in Mapviz
1. Run the pathMgr with `rosrun rover_nav pathMgr.py`
2. Change the content of gpsGoals.txt to the desired coordinates and save.
3. Call the service with `rosservice call /createPath`
4. The path should now display on the map. Recall the service anytime you modify gpsGoals.txt to apply the changes.

### Setting waypoint as a goal
1. Make sure pathMgr is running.
2. Call the service with `rosservice call /setWaypoint <waypointNumber>`

# Installing geographiclib
1. pip install geographiclib

# Installing robot_localization
1. Download the source code https://github.com/cra-ros-pkg/robot_localization/releases/tag/2.6.5
2. Extract to /{workspace}/src/
3. run `rospack profile`in a terminal to to force an update of the package cache
4. build by going to the root of the worspace and using the command `catkin_make`
