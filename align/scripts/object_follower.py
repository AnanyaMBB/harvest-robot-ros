#!/usr/bin/env python
import rospy
from detection_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import time

class ObjectFollower:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.bounding_box_sub = rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.bounding_box_callback)
        self.proximity_sub = rospy.Subscriber('/proximity', Int32, self.proximity_callback)

        # These parameters may need adjustment or could be set via ROS params
        self.image_width = rospy.get_param('image_width', 640)   # default is 640
        self.image_height = rospy.get_param('image_height', 480) # default is 480
        self.speed = 0.01
        self.turning_factor = 0.001
        self.tolerance = 100
        self.min_distance = 10  # minimum distance to move forward (may need adjustment)
        self.current_distance = None

    def bounding_box_callback(self, msg):
        # Assume the bounding boxes are sorted by their size or relevance
        # This could be adjusted as per your setup
        for box in msg.bounding_boxes:
            # Check if the object is ripe
            print(box.Class)
            if box.Class in ["freshripe", "ripe"]:
                # Calculate object offset
                x_center = box.xmin + (box.xmax - box.xmin) / 2
                y_center = box.ymin + (box.ymax - box.ymin) / 2
                offset = x_center - self.image_width / 2
                
                # Publish velocity commands
                twist = Twist()

                # If the offset is less than the tolerance and the distance is large enough, move forward
                print(offset, self.current_distance)
                if abs(offset) < self.tolerance and self.current_distance and self.current_distance > self.min_distance:
                    self.speed = 0.1
                    self.turning_factor = 0.0
                elif self.current_distance < self.min_distance:
                    self.speed = 0.0
                    self.turning_factor = 0.0

                    twist.linear.x = self.speed
                    twist.angular.z = -offset * self.turning_factor
                    self.cmd_vel_pub.publish(twist)

                    time.sleep(1)

                    twist.linear.x = -2 * 0.1
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)

                    time.sleep(1)

                    self.speed = 0.0
                    self.turning_factor = 0.0
                else:
                    self.speed = 0.1
                    self.turning_factor = 0.001

                twist.linear.x = self.speed
                twist.angular.z = -offset * self.turning_factor
                self.cmd_vel_pub.publish(twist)

                # Stop processing further bounding boxes
                break

    def proximity_callback(self, msg):
        # Update current distance
        self.current_distance = msg.data

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('object_follower')
    print('Initiating connection')
    follower = ObjectFollower()
    follower.spin()
