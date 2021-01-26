import cv2


def bgr8_to_jpeg(value, quality=75):
    return cv2.imencode('.jpg', value)[1]

def np_to_CvColor(np_array):
    return cv2.imdecode(np_array, cv2.IMREAD_COLOR)
