from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist
from filterpy.kalman import KalmanFilter
import numpy as np

class ObjectTracker:

    def __init__(self, gating_threshold, kalman_R, kalman_Q):
        self.gating_threshold = gating_threshold
        self.kalman_R = kalman_R
        self.kalman_Q = kalman_Q
        self.tracked_objects = []


    def hungarian_assigner(self, tracked_object_poses, detections):
        # Calculate the cost matrix
        cost = cdist(tracked_object_poses,detections)
        # Run Hungarian assignment
        row_ind, col_ind = linear_sum_assignment(cost)
        
        assignment_costs = np.array([cost[i,j] for i,j in zip(row_ind, col_ind)])
        assignments = np.array([[i,j] for i,j in zip(row_ind, col_ind)]).reshape(-1,2)

        return assignment_costs, assignments

    def initialize_new_tracked_object(self, non_gated_detections):

        for new_object in non_gated_detections:
            self.tracked_objects.append(self.initialize_kalman_filter(new_object))

    def initialize_kalman_filter(self, initial_position):
        f = KalmanFilter(dim_x=3, dim_z=3)
        f.x = initial_position
        
        f.F = np.eye(3) # Object is static
        f.H = np.eye(3) # Observations are in Odometery Frame

        # Tunable Parameters
        f.R *= self.kalman_R
        f.P *= f.R
        f.Q *= self.kalman_Q

        return f

    def run(self, detections):

        # Make Prediction
        for tracked_object in self.tracked_objects:
            tracked_object.predict()

        ## Run Hungarian Assigner
        tracked_object_poses = np.array([f.x for f in self.tracked_objects]).reshape((-1,3))
        assignment_costs, assignments = self.hungarian_assigner(tracked_object_poses, detections)

        ## Find the gated objects
        gated_assignments = np.where(assignment_costs <= self.gating_threshold)[0]
        non_gated_assignments = ~np.isin(np.array(range(detections.shape[0])), assignments[gated_assignments,1])
    
        ## Update their states with Kalman
        for idx in gated_assignments:
            tracked_object_idx, detection_idx =  assignments[idx]
            self.tracked_objects[tracked_object_idx].update(detections[detection_idx])

        ## Initialize new detections
        self.initialize_new_tracked_object(detections[non_gated_assignments])

        ## Predict the position of all objects and return
        return np.array([f.x for f in self.tracked_objects])