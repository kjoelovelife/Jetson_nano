import numpy as np
from . import logger
from duckietown_utils.expand_variables import expand_environment
import os

__all__ = [
    'd8n_read_images_interval',
    'd8n_read_all_images',
    'd8n_get_all_images_topic',
]

def d8n_read_images_interval(filename, t0, t1):
    """
        Reads all the RGB data from the bag,
        in the interval [t0, t1], where t0 = 0 indicates
        the first image.

    """
    data = d8n_read_all_images(filename, t0, t1)
    logger.info('Read %d images from %s.' % (len(data), filename))
    timestamps = data['timestamp']
    # normalize timestamps
    first = data['timestamp'][0]
    timestamps -= first
    logger.info('Sequence has length %.2f seconds.' % timestamps[-1])
    return data

def d8n_read_all_images(filename, t0=None, t1=None):
    """

        Raises a ValueError if not data could be read.

        Returns a numpy array.

        data = d8n_read_all_images(bag)

        print data.shape # (928,)
        print data.dtype # [('timestamp', '<f8'), ('rgb', 'u1', (480, 640, 3))]

    """
    import rosbag  # @UnresolvedImport
    filename = expand_environment(filename)
    if not os.path.exists(filename):
        msg = 'File does not exist: %r' % filename
        raise ValueError(msg)
    bag = rosbag.Bag(filename)
    that_topic = get_image_topic(bag)

    data = []
    first_timestamp = None
    with rosbag.Bag(filename, 'r') as bag:
        for j, (topic, msg, t) in enumerate(bag.read_messages()):
            if topic == that_topic:
                float_time = t.to_sec()
                if first_timestamp is None:
                    first_timestamp = float_time

                rel_time = float_time - first_timestamp
                if t0 is not None:
                    if rel_time < t0:
                        continue
                if t1 is not None:
                    if rel_time > t1:
                        continue
                rgb = numpy_from_ros_compressed(msg)
                data.append({'timestamp': float_time, 'rgb': rgb})

                if j % 10 == 0:
                    print('Read %d images from topic %s' % (j, topic))

    print('Returned %d images' % len(data))
    if not data:
        raise ValueError('no data found')

    H, W, _ = rgb.shape  # (480, 640, 3)
    print('Detected image shape: %s x %s' % (W, H))
    n = len(data)
    dtype = [
        ('timestamp', 'float'),
        ('rgb', 'uint8', (H, W, 3)),
    ]
    x = np.zeros((n,), dtype=dtype)
    for i, v in enumerate(data):
        x[i]['timestamp'] = v['timestamp']
        x[i]['rgb'][:] = v['rgb']

    return x

def d8n_get_all_images_topic(bag_filename):
    """ Returns the (name, type) of all topics that look like images """
    import rosbag  # @UnresolvedImport
    bag = rosbag.Bag(bag_filename)
    tat = bag.get_type_and_topic_info()
# TypesAndTopicsTuple(msg_types={'sensor_msgs/Image': '060021388200f6f0f447d0fcd9c64743', 'dynamic_reconfigure/ConfigDescript
# ion': '757ce9d44ba8ddd801bb30bc456f946f', 'diagnostic_msgs/DiagnosticArray': '60810da900de1dd6ddd437c3503511da', 'rosgraph_
# msgs/Log': 'acffd30cd6b6de30f120938c17c593fb', 'sensor_msgs/CameraInfo': 'c9a58c1b0b154e0e6da7578cb991d214', 'duckietown_ms
# gs/CarControl': '8cc92f3e13698e26d1f14ab2f75ce13b', 'theora_image_transport/Packet': '33ac4e14a7cff32e7e0d65f18bb410f3', 'd
# ynamic_reconfigure/Config': '958f16a05573709014982821e6822580', 'sensor_msgs/Joy': '5a9ea5f83505693b71e785041e67a8bb'}, top
# ics={'/rosberrypi_cam/image_raw/theora/parameter_updates': TopicTuple(msg_type='dynamic_reconfigure/Config', message_count=
# 1, connections=1, frequency=None), '/rosberrypi_cam/image_raw/theora': TopicTuple(msg_type='theora_image_transport/Packet',
#  message_count=655, connections=1, frequency=8.25919467858131), '/rosout': TopicTuple(msg_type='rosgraph_msgs/Log', message
# _count=13, connections=6, frequency=23763.762039660058), '/rosberrypi_cam/camera_info': TopicTuple(msg_type='sensor_msgs/Ca
# meraInfo', message_count=649, connections=1, frequency=8.231283478868388), '/rosberrypi_cam/image_raw/theora/parameter_desc
# riptions': TopicTuple(msg_type='dynamic_reconfigure/ConfigDescription', message_count=1, connections=1, frequency=None), '/
# joy': TopicTuple(msg_type='sensor_msgs/Joy', message_count=1512, connections=1, frequency=182.16304017372423), '/rosout_agg
# ': TopicTuple(msg_type='rosgraph_msgs/Log', message_count=2, connections=1, frequency=12122.265895953757), '/diagnostics':
# TopicTuple(msg_type='diagnostic_msgs/DiagnosticArray', message_count=65, connections=1, frequency=0.9251713284731169), '/ca
# r_supervisor/car_control': TopicTuple(msg_type='duckietown_msgs/CarControl', message_count=3886, connections=1, frequency=5
# 0.09799097011538), '/joy_mapper/joy_control': TopicTuple(msg_type='duckietown_msgs/CarControl', message_count=3881, connect
# ions=1, frequency=50.10517261975869), '/rosberrypi_cam/image_raw': TopicTuple(msg_type='sensor_msgs/Image', message_count=6
# 45, connections=1, frequency=7.711386340215899)}
    consider_images = [
        'sensor_msgs/Image',
        'sensor_msgs/CompressedImage',
    ]
    all_types = set()
    found = []
    topics = tat.topics
    for t,v in topics.items():
        msg_type = v.msg_type
        all_types.add(msg_type)
        message_count = v.message_count
        if msg_type in consider_images:

            # quick fix: ignore image_raw if we have image_compressed version
            if 'raw' in t:
                other = t.replace('raw', 'compressed')

                if other in topics:
                    continue
            found.append((t,msg_type))

    print('all_types: %s' % all_types)
    print('found: %s' % found)
    return found

def get_image_topic(bag):
    """ Returns the name of the topic for the main camera """
    topics = bag.get_type_and_topic_info()[1].keys()
    for t in topics:
        if 'camera_node/image/compressed' in t:
            return t
    msg = 'Cannot find the topic: %s' % topics
    raise ValueError(msg)

def numpy_from_ros_compressed(msg):
    if 'CompressedImage' in msg.__class__.__name__:
        return rgb_from_pil(pil_from_CompressedImage(msg))
    assert False, msg.__class__.__name__

def pil_from_CompressedImage(msg):
    from PIL import ImageFile  # @UnresolvedImport
    parser = ImageFile.Parser()
    parser.feed(msg.data)
    res = parser.close()
    return res

def rgb_from_pil(im):
    return np.asarray(im).astype(np.uint8)
