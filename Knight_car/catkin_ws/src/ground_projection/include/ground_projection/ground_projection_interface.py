from collections import namedtuple
import os.path

import cv2

from duckietown_msgs.msg import Segment, SegmentList
import duckietown_utils as dtu
from ground_projection.configuration import get_extrinsics_filename
import numpy as np
from pi_camera import get_camera_info_for_robot

from .configuration import get_homography_for_robot
from .ground_projection_geometry import GroundProjectionGeometry

__all__ = [
    'GroundProjection',
]


@dtu.memoize_simple
def get_ground_projection(robot_name):
    return GroundProjection(robot_name)


@dtu.contract(returns=GroundProjectionGeometry, robot_name=str)
def get_ground_projection_geometry_for_robot(robot_name):
    gp = get_ground_projection(robot_name)
    return gp.get_ground_projection_geometry()


class GroundProjection(object):

    def __init__(self, robot_name):
        camera_info = get_camera_info_for_robot(robot_name)
        homography = get_homography_for_robot(robot_name)
        self._gpg = GroundProjectionGeometry(camera_info, homography)

        self.robot_name = robot_name

    @dtu.contract(returns=GroundProjectionGeometry)
    def get_ground_projection_geometry(self):
        return self._gpg

    def get_camera_info(self):
        return self._gpg.ci


class CouldNotCalibrate(Exception):
    pass


HomographyEstimationResult = namedtuple('HomographyEstimationResult',
                                        'success error board_info bgr_detected bgr_detected_refined H')


def estimate_homography(bgr_rectified):
    '''
        Estimate ground projection using instrinsic camera calibration parameters.

        Returns HomographyEstimationResult
    '''

    assert bgr_rectified.shape == (480, 640, 3)
    grey_rectified = cv2.cvtColor(bgr_rectified, cv2.COLOR_BGR2GRAY)

    board = load_board_info()
    board_width = board['width']
    board_height = board['height']
    square_size = board['square_size']
    board_offset = board['offset']
    dtu.logger.info('board: \n %s' % board)

    # Defaults
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH
    #+ cv2.CALIB_CB_NORMALIZE_IMAGE
    #flags = cv2.CALIB_CB_NORMALIZE_IMAGE
    #flags = cv2.CALIB_CB_NORMALIZE_IMAGE

    pattern = (board_width, board_height)
    the_input = grey_rectified.copy()
    ret, corners = cv2.findChessboardCorners(the_input, pattern, flags=flags)

#    bgr_detected = cv2.cvtColor(grey_rectified, cv2.COLOR_GRAY2BGR)
    bgr_detected = bgr_rectified.copy()
    cv2.drawChessboardCorners(bgr_detected, (7, 6), corners, ret)

    if ret == False:
        msg = "findChessboardCorners failed (len(corners) == %s)" % (len(corners) if corners is not None else 'none')
        return HomographyEstimationResult(success=False,
                                          error=msg,
                                          board_info=board,
                                          bgr_detected=bgr_detected,
                                          bgr_detected_refined=None,
                                          H=None)

    expected = board_width * board_height
    if len(corners) != expected:
        msg = "Not all corners found in image. Expected: %s; found: %s" % (expected, len(corners))
        dtu.raise_desc(CouldNotCalibrate, msg)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)

    corners2 = cv2.cornerSubPix(grey_rectified, corners, (11, 11), (-1, -1), criteria)

#    bgr_detected_refined = cv2.cvtColor(grey_rectified, cv2.COLOR_GRAY2BGR)
    bgr_detected_refined = grey_rectified.copy()
    cv2.drawChessboardCorners(bgr_detected_refined, (7, 6), corners2, ret)

    src_pts = []
    for r in range(board_height):
        for c in range(board_width):
            src_pts.append(np.array([r * square_size , c * square_size] , dtype='float32')
                           + board_offset)

    # OpenCV labels corners left-to-right, top-to-bottom
    # We're having a problem with our pattern since it's not rotation-invariant

    # only reverse order if first point is at bottom right corner
    if ((corners[0])[0][0] < (corners[board_width * board_height - 1])[0][0] and \
        (corners[0])[0][0] < (corners[board_width * board_height - 1])[0][1]):
        dtu.logger.info("Reversing order of points.")
        src_pts.reverse()

    # Compute homography from image to ground
    H, _mask = cv2.findHomography(corners2.reshape(len(corners2), 2), np.array(src_pts), cv2.RANSAC)

    return HomographyEstimationResult(success=True,
                                      error=None,
                                      board_info=board,
                                      bgr_detected=bgr_detected,
                                      bgr_detected_refined=bgr_detected_refined,
                                      H=H)


def save_homography(H, robot_name):
    dtu.logger.info('Homography:\n %s' % H)

    # Check if specific point in matrix is larger than zero (this would definitly mean we're having a corrupted rotation matrix)
    if(H[1][2] > 0):
        msg = "WARNING: Homography could be corrupt."
        msg += '\n %s' % H
        raise Exception(msg)

    ob = {'homography': sum(H.reshape(9, 1).tolist(), [])}

    import yaml as alt
    s = alt.dump(ob)
    s += "\n# Calibrated on dtu.format_time_as_YYYY_MM_DD(time.time())"

    fn = get_extrinsics_filename(robot_name)

    dtu.write_data_to_file(s, fn)

#    dtu.yaml_write_to_file(ob, extrinsics_filename)


@dtu.contract(sl=SegmentList, gpg=GroundProjectionGeometry, returns=SegmentList)
def find_ground_coordinates(gpg, sl, skip_not_on_ground=True):
    """
        Creates a new segment list with the ground coordinates set.

    """
    cutoff = 0.01
    sl2 = SegmentList()
    sl2.header = sl.header

    # Get ground truth of segmentList
    for s1 in sl.segments:
        g0 = gpg.vector2ground(s1.pixels_normalized[0])
        g1 = gpg.vector2ground(s1.pixels_normalized[1])
        if skip_not_on_ground:
            if g0.x < cutoff or g1.x < cutoff:
                continue

        points = [g0, g1]
        pixels_normalized = [s1.pixels_normalized[0], s1.pixels_normalized[1]]
        color = s1.color
        s2 = Segment(points=points, pixels_normalized=pixels_normalized, color=color)
        # TODO what about normal and points
        sl2.segments.append(s2)
    return sl2


def load_board_info(filename=None):
    '''Load calibration checkerboard info'''
    if filename is None:
        root = dtu.get_ros_package_path('duckietown')
        filename = root + '/config/baseline/ground_projection/ground_projection/default.yaml'

    if not os.path.isfile(filename):
        msg = 'No such file: %s' % filename
        raise dtu.DTException(msg)

    target_data = dtu.yaml_load_file(filename)
    target_info = {
        'width': target_data['board_w'],
        'height': target_data['board_h'],
        'square_size': target_data['square_size'],
        'x_offset': target_data['x_offset'],
        'y_offset': target_data['y_offset'],
        'offset': np.array([target_data['x_offset'], -target_data['y_offset']]),
        'size': (target_data['board_w'], target_data['board_h']),
      }
    dtu.logger.info("Loaded checkerboard parameters")
    return target_info

