#!/usr/bin/env python
# coding: utf-8

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.node_name = "fllower_node"
    rospy.init_node(self.node_name)
    
    self.bridge = cv_bridge.CvBridge()
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    self.del_time = 0.0
    self.integral = 0.0
    self.per_error = 0.0
    self.per_time = rospy.Time().now().to_sec()
    # rospy.sleep(1)
    rospy.on_shutdown(self.cleanup)

  def image_callback(self, msg):
    # 利用cv_bridge把/camera/rgb/image_raw话题消息的sensor_msgs/Image数据类型转化为OpenCV的图像类型
    image = self.bridge.imgmsg_to_cv2(msg)
    
    height, width, channels = image.shape

    crop_img = image[200:height][0:width]


    # 把OpenCV的BGR图像转换为HSV图像
    hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
    
    # 提取黄色车道
    lower_yellow = numpy.array([50,255,255])
    upper_yellow = numpy.array([90,255,255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # cv2.imshow("window", mask) 
    # cv2.waitKey(3)
    res = cv2.bitwise_and(crop_img,crop_img,mask = mask)

     


    m = cv2.moments(mask,False)
    cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
    cv2.circle(res , (int(cx) , int(cy)) , 5 , (0,0,255 ), 2)

    cv2.imshow("window", res )
    cv2.waitKey(2)

    error_x = cx - width / 2
    now_time = rospy.Time().now().to_sec()
    self.del_time = now_time - self.per_time

    angle_error = (0.8 * error_x + 0.1 * (error_x - self.per_error)/self.del_time)/360
    
    self.per_error = error_x
    self.per_time = now_time

    twist_object = Twist()
    twist_object.linear.x = 1
    twist_object.angular.z = -angle_error
    
    rospy.loginfo("ANGULAR VALUE SENT ===>"+str(twist_object.angular.z))
    self.cmd_vel_pub.publish(twist_object)


  def cleanup(self):
     
    rospy.loginfo("Stopping the robot...")
    self.cmd_vel_pub.publish(Twist())


    

if __name__ == '__main__':
    
  f = Follower()
  rospy.spin()

