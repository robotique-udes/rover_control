#!/usr/bin/env python

import os
import glob
import shutil
import subprocess
import cv2
import rospy
import piexif
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_srvs.srv import Empty
from datetime import datetime

# TODO: Place pictures in new folder after having stitched

def load_image(filename):
  img = cv2.imread(filename, -1)
  if img is None:
    rospy.logerr("ERROR image file didn't load or doesn't exist: %s" % filename)
    exit(1)
  return img

class HuginPanorama():
  def __init__(self):
    self.take_pic = False
    self.image_subfolder = None
    self.image_subfolder_path = None
    self.output_image_name = "output"
    self.image_counter = 1
    self.bridge = CvBridge()
    self.camera_info = None

    # Get the path to where input and output images for the panorama are stored
    self.images_path = rospy.get_param('~images_path')

    # Create path if it does not exist
    if not os.path.exists(self.images_path):
        os.makedirs(self.images_path)

    rospy.loginfo('Using working directory: %s'% self.images_path)

    rospy.Service('~stitch', Empty, self.stitch_callback)
    rospy.Service('~reset', Empty,  self.reset_callback)
    rospy.Service('~save_image', Empty, self.save_image_callback)
    rospy.Subscriber('image', Image, self.image_callback)
    rospy.Subscriber('camera_info', CameraInfo, self.camera_info_callback)
    self.publisher = rospy.Publisher('panorama/compressed', Image, queue_size=10)  # FIXME: need non compressed topic to publish compressed

  def get_date_time_stamp(self):
    # Get date and time stamp in a formated string
    date = datetime.now()
    return date.strftime("%Y-%m-%d-%H-%M-%S")

  def save_image(self):
    # Raises flag to take picture and waits until it is taken
    self.take_pic = True
    r = rospy.Rate(10)
    while(self.take_pic):
      # Wait for picture to be taken
      # TODO: add timeout
      r.sleep()
    return True

  def save_image_callback(self, data):
    self.save_image()
    return []

  def stitch_callback(self, data):
    self.do_stitch()
    return []

  def reset_callback(self, data):
    rospy.loginfo('Clearing images waiting to be stitched.')
    self.cleanup(delete_images=True)
    return []

  def image_callback(self, msg):
    if(self.take_pic):
      rospy.loginfo("Saving image")
      if self.image_subfolder == None:
        self.image_subfolder = self.get_date_time_stamp()
        self.image_subfolder_path = "%s/%s" % (self.images_path, self.image_subfolder) 
      if not os.path.exists(self.image_subfolder_path):
        os.makedirs(self.image_subfolder_path)
      try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Save your OpenCV2 image as a jpeg 
        name = "%s/%s/%03d" % (self.images_path, self.image_subfolder, self.image_counter)
        image_name = name + ".jpg"
        camera_info_name = name + ".ini"
        cv2.imwrite(image_name, cv2_img)
        self.save_camera_info(camera_info_name)
        rospy.loginfo("Image saved to %s" % name)
        exif_ifd = {piexif.ExifIFD.FocalLength: (35, 1)}
        exif_dict = {"0th": {}, "Exif": exif_ifd, "1st": {},
             "thumbnail": None, "GPS": {}}
        exif_bytes = piexif.dump(exif_dict)
        piexif.insert(exif_bytes, image_name)
        self.image_counter += 1
      except CvBridgeError, e:
          print(e)
      self.take_pic = False

  def save_camera_info(self, path):
    with open(path, 'w') as f:
      f.write("# Camera intrinsics\n\n")
      f.write("[image]\n\n")
      f.write("width\n%d\n\n" % self.camera_info.width)
      f.write("height\n%d\n\n" % self.camera_info.height)
      f.write("[camera]\n\n")
      f.write("camera matrix\n")
      for i in range(3):
        for j in range(3):
          f.write("%.5f " % self.camera_info.K[(i*3)+j])
        f.write("\n")
      f.write("\ndistortion\n")
      for i in range(5):  # FIXME: this only works for the plum_bob distortion model
        f.write("%.5f " % self.camera_info.D[i])
      f.write("\n\n\nrectification\n")
      for i in range(3):
        for j in range(3):
          f.write("%.5f " % self.camera_info.R[(i*3)+j])
        f.write("\n")
      f.write("\nprojection\n")
      for i in range(3):
        for j in range(4):
          f.write("%.5f " % self.camera_info.P[i*4+j])
        f.write("\n")

  def camera_info_callback(self, msg):
    self.camera_info = msg

  def get_image_filenames(self):
    """ returns space seperated list of files in the images_path """
    print(" ".join(os.listdir(self.image_subfolder_path)))
    return " ".join(os.listdir(self.image_subfolder_path))

  def hugin_stitch(self):
    files = self.get_image_filenames()

    if files == None or files == "":
      rospy.logerr("Did not find any images to stitch in: %s" % self.images_path)
      return

    rospy.loginfo("Stitching files: %s" % files)
    
    # create project file
    self.bash_exec('pto_gen -o pano.pto %s' % files)
    # do cpfind
    self.bash_exec('cpfind -o pano.pto --multirow --celeste pano.pto')
    # do clean
    self.bash_exec('cpclean -o pano.pto pano.pto')
    # do vertical lines
    self.bash_exec('linefind -o pano.pto pano.pto')
    # do optimize locations
    self.bash_exec('autooptimiser -a -m -l -s -o pano.pto pano.pto')
    # calculate size
    self.bash_exec('pano_modify --canvas=AUTO --crop=AUTO -o pano.pto pano.pto')
    # stitch
    self.bash_exec('hugin_executor --stitching --prefix=output pano.pto')
    # compress
    self.bash_exec('convert %s.tif %s.jpg' % (self.output_image_name, self.output_image_name))

    output_file = os.path.join(self.image_subfolder_path, self.output_image_name)
    if not os.path.isfile(output_file):
      rospy.logerr('Hugin failed to create a panorama.')
      return

    rospy.loginfo('Panorama saved to: %s/%s' % (self.image_subfolder_path, self.output_image_name))

    self.publish_pano()
    rospy.loginfo('Finished.')

  def publish_pano(self):
    img = load_image(os.path.join(self.output_image_name, self.output_image_name))
    compressed_image = CvBridge().cv2_to_compressed_imgmsg(img)
    self.publisher.publish(compressed_image)

  def cleanup(self, delete_images=False):
    # Hugin project files and output images
    files_to_be_deleted = ['%s.tif' % self.output_image_name, "%s.jpg" % self.output_image_name, 'pano.pto']

    # Optionally delete images
    if delete_images:
      image_types = ('*.jpg', '*.jpg', '*.gif')
      for file_type in image_types:
        path = os.path.join(self.output_image_name,file_type)
        files_to_be_deleted.extend(glob.glob(path))

    # Do file deletion
    for file in files_to_be_deleted:
      file_to_be_deleted = os.path.join(self.output_image_name, file)
      if os.path.isfile(file_to_be_deleted):
        os.remove(file_to_be_deleted)

  def do_stitch(self):
    rospy.loginfo('Stitching panorama...')
    self.cleanup()
    self.hugin_stitch()

  def bash_exec(self, cmd):
    sp = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=self.image_subfolder_path)
    out, err = sp.communicate()
    if out:
        rospy.loginfo("\n%s" % out)
    if err:
        rospy.logerr("\n%s" % err)
    return out, err, sp.returncode

def main():
  rospy.init_node('hugin_panorama')
  Pano = HuginPanorama()
  rospy.spin()
