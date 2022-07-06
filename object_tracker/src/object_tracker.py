from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist
from filterpy.kalman import KalmanFilter
from median_filter import MedianFilter
import numpy as np
import pandas as pd
import os

class TrackedObject:
    
    def __init__(self, filter_):
        self.filter = filter_
        self.number_of_occurrence = 1
    
    def update(self, value):
        self.filter.update(value)
        self.number_of_occurrence+=1
    
    def predict(self):
        self.filter.predict()

class ObjectTracker:

    def __init__(self, gating_threshold, kalman_R, kalman_Q, filter_type):
        self.gating_threshold = gating_threshold
        self.kalman_R = kalman_R
        self.kalman_Q = kalman_Q
        self.filter_type = filter_type
        self.tracked_objects = {}


    def hungarian_assigner(self, tracked_object_poses, detections):
        # Calculate the cost matrix
        cost = cdist(tracked_object_poses,detections)
        # Run Hungarian assignment
        row_ind, col_ind = linear_sum_assignment(cost)
        
        assignment_costs = np.array([cost[i,j] for i,j in zip(row_ind, col_ind)])
        assignments = np.array([[i,j] for i,j in zip(row_ind, col_ind)]).reshape(-1,2)

        return assignment_costs, assignments

    def initialize_new_tracked_object(self, non_gated_detections, class_id):

        for new_object in non_gated_detections:
            self.tracked_objects[class_id].append(TrackedObject(self.initialize_kalman_filter(new_object)))

    def initialize_kalman_filter(self, initial_position):
        if self.filter_type == "kalman":
            f = KalmanFilter(dim_x=3, dim_z=3)
            f.x = initial_position
            
            f.F = np.eye(3) # Object is static
            f.H = np.eye(3) # Observations are in Odometery Frame

            # Tunable Parameters
            f.R *= self.kalman_R
            f.P *= f.R
            f.Q *= self.kalman_Q

        elif self.filter_type == "median" or self.filter_type == "mean":
            f = MedianFilter(initial_position, self.filter_type)

        return f

    def run_class(self, detections, class_id):

        # Make Prediction
        for tracked_object in self.tracked_objects[class_id]:
            tracked_object.predict()

        ## Run Hungarian Assigner
        tracked_object_poses = np.array([f.filter.x for f in self.tracked_objects[class_id]]).reshape((-1,3))
        assignment_costs, assignments = self.hungarian_assigner(tracked_object_poses, detections)

        ## Find the gated objects
        gated_assignments = np.where(assignment_costs <= self.gating_threshold)[0]
        non_gated_assignments = ~np.isin(np.array(range(detections.shape[0])), assignments[gated_assignments,1])
    
        ## Update their states with Kalman
        for idx in gated_assignments:
            tracked_object_idx, detection_idx =  assignments[idx]
            self.tracked_objects[class_id][tracked_object_idx].update(detections[detection_idx])

        ## Initialize new detections
        self.initialize_new_tracked_object(detections[non_gated_assignments], class_id)

    def run(self, detections):
        # Add if new class is detected
        for class_ in detections:
            if class_ not in self.tracked_objects:
                self.tracked_objects[class_] = []

            self.run_class(detections[class_], class_)        

        tracked_objects = {}
        print("-----------------------------------")
        for class_ in self.tracked_objects:
            print(class_, len(self.tracked_objects[class_]))
            tracked_objects[class_] = {
                "poses" : np.array([f.filter.x for f in self.tracked_objects[class_]]).reshape(-1,3),
                "num_of_occur" : np.array([f.number_of_occurrence for f in self.tracked_objects[class_]])
            }
        print("-----------------------------------")

        ObjectTracker.generate_csv(tracked_objects)

        return tracked_objects

    @staticmethod
    def generate_csv(tracked_objects):

        data = {
            "class": [],
            "x": [],
            "y": [],
            "z": [],
            "num_occur": []
        }
        for class_ in tracked_objects:
            for object, num_occur in zip(tracked_objects[class_]["poses"],tracked_objects[class_]["num_of_occur"]):
                data["class"].append(class_)
                data["x"].append(object[0])
                data["y"].append(object[1])
                data["z"].append(object[2])
                data["num_occur"].append(num_occur)
        
        df = pd.DataFrame(data)

        output_path  = os.path.join(os.path.abspath( os.path.dirname( __file__ ) ), "../output/object_detections.csv")
        df.to_csv(output_path, index=False)
