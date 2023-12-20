import cv2

from marker import Marker


class Aruco(Marker):
    def __init__(self, marker_id, camera):
        super().__init__(marker_id, camera)
        self.aruco_dict = self._get_marker_dict()

    def _get_marker_dict(self):
        """
        Get the dictionary of the marker.

        :return: The dictionary of the marker.
        """
        aruco_dict = cv2.aruco.DICT_4X4_250
        return cv2.aruco.getPredefinedDictionary(aruco_dict)

    def detect_markers(self):
        """
        Detect the markers.

        :return: The corners and ids of the markers.
        """
        frame_gray = self.frame_gray_return()
        corners, ids, _ = cv2.aruco.detectMarkers(frame_gray, self.aruco_dict)
        list_ids = ids.tolist()
        return corners, list_ids

    def return_rvecs_tvecs(self, corners):
        """
        Return the rvecs and tvecs of the markers.
        :param corners: The corners of the markers.

        :return: The rvecs and tvecs of the markers.
        """
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners=corners,
            markerLength=self.marker_size,
            cameraMatrix=self.camera_mtx,
            distCoeffs=self.camera_dist)
        return rvecs, tvecs