#!/usr/bin/env python3  
from multiprocessing.connection import Listener
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

class Relocalize:
    def __init__(self):
        rospy.init_node('challenge_tf_broadcaster')
        self.received = 0
        self.initialpose_subscriber = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.get_initial_pose_cb)
        self.new_tf = PoseStamped()
        self.br = tf.TransformBroadcaster()
        pass

    def get_initial_pose_cb(self,initial_pose_stamped):
        if self.received < 2:
            self.received += 1
            if self.received == 2:
                self.new_tf.pose.position = initial_pose_stamped.pose.pose.position
                self.new_tf.pose.orientation = initial_pose_stamped.pose.pose.orientation
                rospy.loginfo('Get new tf from 2D pose estimate. Publishing...')
        return 0

    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if self.received >= 2:
                self.br.sendTransform((self.new_tf.pose.position.x, self.new_tf.pose.position.y, self.new_tf.pose.position.z),
                                    (self.new_tf.pose.orientation.x, self.new_tf.pose.orientation.y, self.new_tf.pose.orientation.z, self.new_tf.pose.orientation.w),
                                    rospy.Time.now(),
                                    "challenge_tf",
                                    "map")
            rate.sleep()
    

if __name__ == '__main__':
    rc = Relocalize()
    rc.run()
        
