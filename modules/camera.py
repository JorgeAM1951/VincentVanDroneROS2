import numpy as np
from scipy.spatial.transform import Rotation as R
class Camera:
    def __init__(self, pose, orientation, fov, focal, pixelSize, resolution,):
        self.pose = pose                #[X, Y, Z]
        self.orientation = orientation  #Formato roll, pitch, yaw
        self.fovX = fov[0]              #[fovX, fovY]
        self.fovY = fov[1]
        self.focalX = focal[0]          #[focalX, focalY]
        self.focalY = focal[1]
        self.pixelSizeX = pixelSize[0]  #Tamaño del pixel en metros
        self.pixelSizeY = pixelSize[1]
        self.resolution = resolution    #Resolucion de la camara [1920, 1080], p.e.
        self.imageCenter = [resolution[0]/2, resolution[1]/2]


    def getP(self, scaleFactor=1):
        #Hallamos la matriz de parámetros K
        K = scaleFactor * np.array([self.focalX/self.pixelSizeX, 0, self.imageCenter[0]], 
                               [0, self.focalY/self.pixelSizeY, self.imageCenter[1]], 
                               [0, 0, 1])
        #Hallamos la matriz de rotación (pitch, roll, yaw)
        rot = R.from_euler('x', self.orientation[1], degrees=True)
        RX = rot.as_matrix()
        rot = R.from_euler('y', self.orientation[0], degrees=True)
        RY = rot.as_matrix()
        rot = R.from_euler('z', self.orientation[2], degrees=True)
        RZ = rot.as_matrix()
        RWC = RZ * RY * RX
        #Usamos la R para hallar TWC
        TWC = np.eye(4)
        TWC[:3,:3] = RWC
        TWC[3,:3] = self.pose
        TCW = np.linalg.inv(TWC)
        canonical = np.hstack((np.eye(3),np.zeros(3)))
        P = K @ np.linalg.solve(TWC.T, canonical.T).T #Resuelve XT = C --> X = C*T^-1 --> X = C/T
        return P
