import cv2
import numpy as np
from duckietown_utils import logger
#from PIL import Image as pimg

def image_cv_from_jpg(data):
    """ Returns an OpenCV BGR image from a string """
    s = np.fromstring(data, np.uint8)
    image_cv = cv2.imdecode(s, cv2.IMREAD_COLOR)
    if image_cv is None:
        msg = 'Could not decode image (cv2.imdecode returned None). '
        msg += 'This is usual a sign of data corruption.'
        raise ValueError(msg)
    return image_cv

# class Storage:
#     dst = None
# 
# def image_cv_from_jpg_buf(data):
#     pass
#     """ Returns an OpenCV BGR image from a string """
#     s = np.fromstring(data, np.uint8)
#     if Storage.dst is not None:
#         image_cv = cv2.imdecode(s, cv2.CV_LOAD_IMAGE_COLOR, dst=Storage.dst)
#     else:
#         image_cv = cv2.imdecode(s, cv2.CV_LOAD_IMAGE_COLOR)
#     Storage.dst = image_cv
#     return image_cv


def image_cv_from_jpg_fn(fn):
    with open(fn) as f:
        return image_cv_from_jpg(f.read())


# Second option: use PIL

    
import numpy as np
from PIL import ImageFile  # @UnresolvedImport
def rgb_from_jpg_by_PIL(data):
    """ Warning: this returns RGB """
    parser = ImageFile.Parser()
    parser.feed(data)
    res = parser.close() 
    res = np.asarray(res)
    return res

# third option: jpeg library
import StringIO



def rgb_from_jpg_by_JPEG_library(data):
    try:
        import jpeg4py as jpeg
    except ImportError as e:
        installation = """
sudo apt-get install -y libturbojpeg  python-cffi
sudo pip install jpeg4py
"""
        logger.error(installation)
        raise

    jpg_data = np.fromstring(data, dtype=np.uint8)
    image_cv = jpeg.JPEG(jpg_data).decode()
    return image_cv


def image_clip_255(image_float):
    """ Clips to 0,255 and converts to uint8 """
    h,w,_ = image_float.shape
    res = np.zeros((h,w,3), dtype=np.uint8)
    np.clip(image_float, 0, 255, out=res)
    return res
    
#     
# def imgmsg_from_cv2(image_cv):
#     return 
#         self.corrected_image = self.bridge.cv2_to_imgmsg(corrected_image_cv2,"bgr8"



# with libjpeg-turbo
# Convert from uncompressed image message
# image_cv = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")




# @contract(images='list[>=1](array)')
def make_images_grid(images, cols=None, pad=0, bgcolor=[1, 1, 1]):
    n = len(images)
    if cols is None:
        cols = int(np.ceil(np.sqrt(n)))

    rows = int(np.ceil(n * 1.0 / cols))

    assert cols > 0 and rows > 0
    assert n <= cols * rows

    # find width and height for the grid 
    col_width = np.zeros(cols, dtype='int32')
    row_height = np.zeros(rows, dtype='int32')
    for i in range(n):
        image = images[i]
        col = i % cols
        row = (i - i % cols) / cols
        assert 0 <= col < cols
        assert 0 <= row < rows

        if pad > 0:
            image = image_border(image,
                       left=pad,
                       right=pad,
                       top=pad,
                       bottom=pad,
                       color=bgcolor)
        width = image.shape[1]
        height = image.shape[0]

        col_width[col] = max(width, col_width[col])
        row_height[row] = max(height, row_height[row])

    canvas_width = sum(col_width)
    canvas_height = sum(row_height)

    # find position for each col and row
    col_x = np.zeros(cols, dtype='int32')
    for col in range(1, cols):
        col_x[col] = col_x[col - 1] + col_width[col - 1]

    assert(canvas_width == col_x[-1] + col_width[-1])

    row_y = np.zeros(rows, dtype='int32')
    for row in range(1, rows):
        row_y[row] = row_y[row - 1] + row_height[row - 1]
    assert(canvas_height == row_y[-1] + row_height[-1])

    canvas = np.zeros((canvas_height, canvas_width, 3), dtype='uint8')
    for k in range(3):
        canvas[:, :, k] = bgcolor[k] * 255

    for i in range(n):
        col = i % cols
        row = (i - i % cols) / cols
        assert 0 <= col < cols
        assert 0 <= row < rows
        image = images[i]
        x = col_x[col]
        y = row_y[row]
        
        # Pad if not right shape
        extra_hor = col_width[col] - image.shape[1]
        extra_ver = row_height[row] - image.shape[0]
        eleft = extra_hor / 2
        eright = extra_hor - eleft
        etop = extra_ver / 2
        ebottom = extra_ver - etop
        image = image_border(image, left=eleft, right=eright, top=etop,
                             bottom=ebottom, color=bgcolor)
        
        # TODO: align here
        place_at(canvas, image, x, y)

    return canvas


def rgb_pad(height, width, color):
    pad = np.zeros((height, width, 3), dtype='uint8')
    for i in range(3):
        pad[:, :, i] = color[i] * 255
    return pad


def image_border(rgb, left=0, right=0, top=0, bottom=0, color=[1, 1, 1]):
    orig_shape = rgb.shape
    
    if left > 0:
        # note: do this every time because it changes throughout
        height, width = rgb.shape[0:2]
        pad = rgb_pad(height, left, color)
        rgb = np.hstack((pad, rgb))

    if right > 0:
        height, width = rgb.shape[0:2]
        pad = rgb_pad(height, right, color)
        rgb = np.hstack((rgb, pad))

    if top > 0:
        height, width = rgb.shape[0:2]
        pad = rgb_pad(top, width, color)
        rgb = np.vstack((pad, rgb))
        assert rgb.shape[0] == height + top

    if bottom > 0:
        height, width = rgb.shape[0:2]
        pad = rgb_pad(bottom, width, color)
        rgb = np.vstack((rgb, pad))
        assert rgb.shape[0] == height + bottom

    assert rgb.shape[0] == orig_shape[0] + top + bottom
    assert rgb.shape[1] == orig_shape[1] + left + right
        
    return rgb



def place_at(canvas, image, xpix, ypix):
    # print canvas.shape, image.shape
    xsize = min(canvas.shape[1] - xpix, image.shape[1])
    ysize = min(canvas.shape[0] - ypix, image.shape[0])
    if len(image.shape) == 2:
        image = image.reshape((image.shape[0], image.shape[1], 1))
    canvas[ypix:(ypix + ysize), xpix:(xpix + xsize), 0:3] = \
        image[0:ysize, 0:xsize, :]


