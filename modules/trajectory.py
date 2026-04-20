class Trajectory:
    def __init__(self, waypoints, period=0.5, degree=7):
        self.waypoints = waypoints
        self.period = period
        self.degree = degree
    
    def timeNormalize(self):
        pass