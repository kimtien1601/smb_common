#!/usr/bin/env python3
import rospy
import numpy as np
from object_detection_msgs.msg import ObjectDetectionInfoArray, ObjectDetectionInfo
import tf2_ros
import tf2_geometry_msgs

from object_tracker import ObjectTracker

class ObjectTrackerROS:
    def __init__(self, fixed_frame_id, object_poses_topic, tracked_object_topic, confidence_thres, gating_threshold, filter_type, kalman_R, kalman_Q):
        # Initialize Subscriber and Publisher
        self.sub = rospy.Subscriber(object_poses_topic, ObjectDetectionInfoArray, self.callback)
        self.pub = rospy.Publisher(tracked_object_topic, ObjectDetectionInfoArray, queue_size=10)
        
        # Initialize TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.fixed_frame_id = fixed_frame_id
        self.confidence_thres = confidence_thres

        # Initialize Object Tracker
        self.object_tracker = ObjectTracker(gating_threshold, kalman_R, kalman_Q, filter_type)

    def transform_object_to_fixed_frame(self, object_pos, object_frame, object_stamp):

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose.position = object_pos
        pose_stamped.pose.orientation.w = 1.0
        pose_stamped.header.frame_id = object_frame
        pose_stamped.header.stamp = object_stamp

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = self.tf_buffer.transform(pose_stamped, self.fixed_frame_id, rospy.Duration(1))
            return np.array([output_pose_stamped.pose.position.x, output_pose_stamped.pose.position.y, output_pose_stamped.pose.position.z])

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise


    def callback(self, msg):
        detections = {}
        for object_ in msg.info:
            if object_.confidence < self.confidence_thres:
                continue
            
            detected_object = self.transform_object_to_fixed_frame(
                object_.position, 
                msg.header.frame_id, 
                msg.header.stamp)
            
            if object_.class_id not in detections:
                detections[object_.class_id] = []

            detections[object_.class_id].append(detected_object)

        for class_id in detections:
            detections[class_id] =  np.array(detections[class_id]).reshape(-1,3)

        tracked_objects = self.object_tracker.run(detections)

        tracked_objects_ros = ObjectDetectionInfoArray()
        tracked_objects_ros.header.stamp = rospy.Time.now()
        tracked_objects_ros.header.frame_id = self.fixed_frame_id

        for class_ in tracked_objects:
            for object, num_of_occur in zip(tracked_objects[class_]["poses"],tracked_objects[class_]["num_of_occur"]) :
                p = ObjectDetectionInfo()
                p.position.x = object[0]
                p.position.y = object[1]
                p.position.z = object[2]

                p.class_id = class_
                p.id = num_of_occur

                tracked_objects_ros.info.append(p)

        self.pub.publish(tracked_objects_ros)

if __name__ == '__main__':
    rospy.init_node('object_tracker')
    
    ObjectTrackerROS(
        "map", # Target Frame
        "/object_detector/detection_info", # Object Poses Topic
        "/object_detector/tracked_objects", # Tracked Objects Topic
        0.6, # Confidence Threshold
        5, # Gating Threshold in Meters
        "kalman", # Filtering Type
        1, # Kalman R
        0.1) # Kalman Q
    
    rospy.spin()



