class Pose():
    def __init__(self, timestep = None, orientationX = None, orientationY = None, orientationZ = None,
                 orientationW = None, positionX = None, positionY = None, positionZ = None):
        self.timestep = timestep
        self.orientationX = orientationX
        self.orientationY = orientationY
        self.orientationZ = orientationZ
        self.orientationW = orientationW

        self.positionX = positionX
        self.positionY = positionY
        self.positionZ = positionZ
