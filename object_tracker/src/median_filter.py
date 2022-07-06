import numpy as np

class MedianFilter:

    def __init__(self, initial_position):
        self.position_array = np.array([initial_position]).reshape(-1,3)
        self.x = initial_position
        
    def update(self, value):
        self.position_array = np.vstack((self.position_array, value))
        self.x = np.median(self.position_array, axis = 0)

    def predict(self):
        pass