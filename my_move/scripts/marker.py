import math
from abc import ABC, abstractmethod
from collections import Counter
from time import time

import cv2
import numpy as np

def calculate_median_vector(vector_collection):
    '''
    Calculates the median vector of a collection of vectors.
    :param vector_collection: A collection of vectors.
    '''
    vector_x_components = [vector[0][0][0] for vector in vector_collection]
    vector_y_components = [vector[0][0][1] for vector in vector_collection]
    vector_z_components = [vector[0][0][2] for vector in vector_collection]

    median_x = np.median(vector_x_components)
    median_y = np.median(vector_y_components)
    median_z = np.median(vector_z_components)

    return np.array([median_x, median_y, median_z])


class Marker(ABC):
    def __init__(self, marker_id, camera, camera_x_offset: float = 0.050,
                 camera_y_offset: float = -0.06475, camera_z_offset: float = -0.070864,):
        '''
        :param marker_id: The id of the marker to be detected.
        :param camera: The camera object.
        :param camera_x_offset: The offset in the x-axis of the camera.
        :param camera_y_offset: The offset in the y-axis of the camera.
        :param camera_z_offset: The offset in the z-axis of the camera.
        '''
        self.frame_count: int = 1
        self.marker_id = marker_id
        self.camera = camera
        self.camera_mtx = self.camera.get_mtx()
        self.camera_dist = self.camera.get_dist()
        self.flag_choice = None
        self.marker_size = 0.02
        self.camera_x_offset = camera_x_offset
        self.camera_y_offset = camera_y_offset
        self.camera_z_offset = camera_z_offset

    @abstractmethod
    def _get_marker_dict(self):
        pass

    def frame_gray_return(self):
        """
        Get the frame in grayscale.

        :return: The grayscale frame.
        """
        frame = self.camera.get_frame()
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return frame_gray

    @abstractmethod
    def detect_markers(self):
        pass

    @abstractmethod
    def return_rvecs_tvecs(self, corners):
        pass

    def rvecs_tvecs_selected(self, list_ids: list = None,
                             rvecs: list = None, tvecs: list = None):
        """
        Get the rvecs and tvecs of the selected marker.
        :param list_ids: The list of ids of the markers.
        :param rvecs: The rvecs of the markers.
        :param tvecs: The tvecs of the markers.

        :return: The rvecs and tvecs of the selected marker.
        """
        try:
            if not self.flag_choice:
                if len(list_ids) == 1:
                    selected_id = list_ids[0][0]
                else:
                    selected_id = self.marker_id
                    self.flag_choice = True

            n = list_ids.index([self.marker_id])

            rvecs_reshaped_array = rvecs[n].reshape((1, 1, 3))
            tvecs_reshaped_array = tvecs[n].reshape((1, 1, 3))

            return rvecs_reshaped_array, tvecs_reshaped_array
        except ValueError:
            raise ValueError("Marker ID not found")

    def id_confirmation(self, list_ids, rvecs, tvecs):
        """
        Confirm the marker.
        :param list_ids: The list of ids of the markers.
        :param rvecs: The rvecs of the markers.
        :param tvecs: The tvecs of the markers.

        :return: The id of the marker.
        """
        distances_list = []

        for marker_cont in range(0, len(list_ids)):
            if abs(np.degrees(rvecs[marker_cont][0][2])) < 20:
                distance_marker = self.marker_distance_calculation(
                    tvecs[marker_cont][0])
            else:
                distance_marker = math.inf

            distances_list.append(distance_marker)

        closest_index = distances_list.index(min(distances_list))
        marker_id_closest = list_ids[closest_index]

        return marker_id_closest[0]

    def marker_distance_calculation(self, tvec: list):
        """
        Calculate the distance of the marker to the gripper.
        :param tvec: The tvec of the marker to the camera.

        :return: The distance of the marker to the gripper.
        """

        x = tvec[0] + self.camera_x_offset
        y = tvec[1] - self.camera_y_offset
        z = tvec[2] + self.camera_z_offset

        distance = math.sqrt(x ** 2 + y ** 2 + z ** 2)
        return distance

    def marker_collections(self):
        """
        Get the rvecs, tvecs and id of the closest marker.

        :return: The rvecs, tvecs and id of the closest marker.
        """
        time_start = time()
        rvecs_collection = []
        tvecs_collection = []
        list_closest = []

        while ((time()) - 5 - time_start) < 5:
            corners, ids = self.detect_markers()

            if len(corners) > 0:

                rvecs, tvecs = self.return_rvecs_tvecs(corners)
                rvecs_reshaped_array, tvecs_reshaped_array = self.rvecs_tvecs_selected(
                    ids, rvecs, tvecs)
                id_closest = self.id_confirmation(ids, rvecs, tvecs)
                list_closest.append(id_closest)

                rvecs_collection.append(rvecs_reshaped_array)
                tvecs_collection.append(tvecs_reshaped_array)

                if len(tvecs_collection) % self.frame_count == 0:
                    break
        return rvecs_collection, tvecs_collection, list_closest

    def marker_average_coordinates(self, rvecs_collection, tvecs_collection):
        """
        Get the average rvecs and tvecs of the markers.
        :param rvecs_collection: The rvecs of the markers.
        :param tvecs_collection: The tvecs of the markers.

        :return: The average rvecs and tvecs of the markers.
        """
        average_rvecs = calculate_median_vector(rvecs_collection)
        average_tvecs = calculate_median_vector(tvecs_collection)
        return average_rvecs, average_tvecs

    def closest_id(self, list_closest):
        """
        Get the closest marker.
        :param list_closest: The list of closest markers.

        :return: The id of the closest marker.
        """
        closest_counter = Counter(list_closest)
        id_closest = -1

        if len(list_closest) > 0:
            id_closest = closest_counter.most_common(1)[0][0]

        if id_closest != self.marker_id:
            print("locale wrong id: ", id_closest)
            return False
        return id_closest

    def marker_pose(self):
        """
        Get the rvecs, tvecs and id of the closest marker.

        :return: The rvecs, tvecs and id of the closest marker.
        """
        try:
            rvecs_collection, tvecs_collection, list_closest = self.marker_collections()
            average_rvecs, average_tvecs = self.marker_average_coordinates(
                rvecs_collection, tvecs_collection)
            id_closest = self.closest_id(list_closest)
        except:
            average_rvecs = []
            average_tvecs = []
            id_closest = None

        return average_rvecs, average_tvecs, id_closest