#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageNode:
  def __init__(self):
    self.topic = rospy.get_param('~image_topic')
    self.angle = rospy.get_param('~image_angle', -180)
    self.scale = rospy.get_param('~image_scale', 1.0)
    self.image_sub = rospy.Subscriber(self.topic, Image, self.callback, queue_size=2)
    self.image_pub = rospy.Publisher(self.topic + '/rotated', Image, queue_size=2)
    self.bridge = CvBridge()

  def callback(self, image):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
      return
    
    (h, w) = cv_image.shape[:2]
    center = (w/2, h/2)
    M = cv2.getRotationMatrix2D(center, self.angle, self.scale)
    rotated = cv2.warpAffine(cv_image, M, (w, h))

    try:
      rotated_img_msg = self.bridge.cv2_to_imgmsg(rotated, "bgr8")
      rotated_img_msg.header = image.header
      self.image_pub.publish(rotated_img_msg)
    except CvBridgeError as e:
      rospy.logerr(e)
      return

if __name__ == '__main__':
  rospy.init_node('image_rotate')
  node = ImageNode()

  try:
      rospy.spin()
  except KeyboardInterrupt:
      rospy.loginfo("Shutting down Image Rotate Node")
  
  cv2.destroyAllWindows()
