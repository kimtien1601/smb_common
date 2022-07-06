import numpy as np

class MedianFilter:

    def __init__(self, initial_position, filter_type):
        self.position_array = np.array([initial_position]).reshape(-1,3)
        self.x = initial_position
        self.filter_type = filter_type
        
    def update(self, value):
        self.position_array = np.vstack((self.position_array, value))
        if self.filter_type == "median":
            self.x = np.median(self.position_array, axis = 0)
        elif self.filter_type == "mean":
            self.x = np.mean(self.position_array, axis = 0)

    def predict(self):
        pass