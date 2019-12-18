# rover_nav dependencies
* mapviz
* docker
* geographiclib

# Mapviz

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

# Autonomous navigation simulation

### Listing waypoint coordinates
Coordinates are listed in `gpsGoals.txt`. Each point must be written on a seperate line. You can write comments on a line or at the end
of a coordinate by prefacing it with a `#`. Comments will be ignored by the parser.

### Changing the rover's starting position
In rover_sim.py, change the values of currentLat and currentLon to any valid wgs84 coordinate.

### Changing the map's origin
In simulation.launch, change the coordinates to any valid wgs84 coordinate. Note that to that the origin should be near the rover's position in order to have the best accuracy.

### Launching the simulation
Open a terminal and enter the following command: `roslaunch rover_nav simulation.launch`
This will launch mapviz along with all other necessary nodes.

### Displaying ideal path in Mapviz
1. Change the content of gpsGoals.txt to the desired coordinates and save.
2. In a different terminal, call the createPath service with `rosservice call /createPath`
3. The path should now display on the map. Recall the service anytime you modify gpsGoals.txt to apply the changes.

### Setting waypoint as a goal
1. Make sure pathMgr is running.
2. In a different terminal, call the setWaypoint service with `rosservice call /setWaypoint <waypointNumber>`
3. The rover will automatically move towards the goal and the navigation will be stopped when the goal is reached.

# Installing geographiclib
1. pip install geographiclib
