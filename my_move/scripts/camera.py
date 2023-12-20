from abc import ABC, abstractmethod
from cv2 import VideoCapture
from numpy import ndarray


class Camera(ABC):
    @abstractmethod
    def __init__(self):
        self.__mtx: ndarray
        self.__dist: ndarray
        self.__cap_number: int
        self.__cv_cap_read = VideoCapture

    @abstractmethod
    def get_cap(self):
        '''
        Get the camera capture.
        '''
        pass

    @abstractmethod
    def release_camera(self):
        '''
        Release the camera.
        '''
        pass

    @abstractmethod
    def get_mtx(self):
        '''
        Get the camera matrix.
        '''
        pass

    @abstractmethod
    def get_dist(self):
        '''
        Get the camera distortion.
        '''
        pass

    @abstractmethod
    def get_frame(self):
        pass