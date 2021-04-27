# hugin_panorama
Fork of hugin_panorama

## Dependencies
* `pip install piexif`
* sudo apt-get install ros-melodic-hugin-panorama hugin-tools enblend

# How to use
1. Change the image topic in hugin_panorama.launch
2. Launch with `roslaunch hugin_panorma hugin_panorama.launch`
3. Make sure your camera node is running
4. Take a pictures with `rosservice call /hugin_panorama/save_image`
5. Stitch the panorama with `rosservice call /hugin_panorama/stitch`
6. By default, the images will be saved in hugin_panorama/image/<timestamp_of_first_image>
