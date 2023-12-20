from cv2 import VideoCapture, CAP_DSHOW
from numpy import float32
from camera import Camera


class Webcam(Camera):

    def __init__(self):
        """
        Initialize the camera.
        """
        self.__mtx = float32([[803.45960464, 0, 304.69672111],
                                 [0, 993.14368468, -112.00094024],
                                 [0, 0, 1]])
        self.__dist = float32([[-0.20757414, 0.79718928, -0.14389031, -0.00596995, -0.64068981]])
        self.__cap_number = 0
        self.__cv_cap_read = VideoCapture(self.__cap_number)

    def get_cap(self):
        return self.__cv_cap_read

    def release_camera(self):
        if self.__cv_cap_read.isOpened():
            self.__cv_cap_read.release()

    def get_mtx(self):
        return self.__mtx

    def get_dist(self):
        return self.__dist

    def get_frame(self):
        ret, frame = self.get_cap().read()
        return frame