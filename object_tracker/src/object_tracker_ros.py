#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Pose

import tf2_ros
import tf2_geometry_msgs

from object_tracker import ObjectTracker

class ObjectTrackerROS:
    def __init__(self, fixed_frame_id, object_poses_topic, tracked_object_topic, gating_threshold, kalman_R, kalman_Q):
        # Initialize Subscriber and Publisher
        self.sub = rospy.Subscriber(object_poses_topic, PoseArray, self.callback)
        self.pub = rospy.Publisher(tracked_object_topic, PoseArray, queue_size=10)
        
        # Initialize TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.fixed_frame_id = fixed_frame_id

        # Initialize Object Tracker
        self.object_tracker = ObjectTracker(gating_threshold, kalman_R, kalman_Q)

    def transform_object_to_fixed_frame(self, object_pose, object_frame, object_stamp):

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = object_pose
        pose_stamped.header.frame_id = object_frame
        pose_stamped.header.stamp = object_stamp

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = self.tf_buffer.transform(pose_stamped, self.fixed_frame_id, rospy.Duration(1))
            return np.array([output_pose_stamped.pose.position.x, output_pose_stamped.pose.position.y, output_pose_stamped.pose.position.z])

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise


    def callback(self, msg):
        detected_objects = []
        for object_pose in msg.poses:
            detected_objects.append(
                self.transform_object_to_fixed_frame(
                    object_pose, 
                    msg.header.frame_id, 
                    msg.header.stamp))

        detections =  np.array(detected_objects).reshape(-1,3)

        tracked_objects = self.object_tracker.run(detections)

        tracked_objects_ros = PoseArray()
        tracked_objects_ros.header.stamp = rospy.Time.now()
        tracked_objects_ros.header.frame_id = self.fixed_frame_id

        for object in tracked_objects:
            p = Pose()
            p.position.x = object[0]
            p.position.y = object[1]
            p.position.z = object[2]

            tracked_objects_ros.poses.append(p)

        self.pub.publish(tracked_objects_ros)

if __name__ == '__main__':
    rospy.init_node('object_tracker')
    
    ObjectTrackerROS(
        "tracking_camera_odom", # Target Frame
        "/object_detector/object_poses", # Object Poses Topic
        "/object_detector/tracked_objects", # Tracked Objects Topic
        0.15, # Gating Threshold in Meters
        1, # Kalman R
        0.1) # Kalman Q
    
    rospy.spin()



