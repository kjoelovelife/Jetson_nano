from duckietown_msgs.msg import Pixel, Segment, SegmentList
import duckietown_utils as dtu
from ground_projection.ground_projection_geometry import GroundProjectionGeometry


@dtu.contract(gpg=GroundProjectionGeometry, s1=Segment, returns=Segment)
def rectify_segment(gpg, s1):
    pixels_normalized = []

    for i in (0, 1):
        # normalized coordinates
        nc = s1.pixels_normalized[i]
        # get pixel coordinates
        pixels = gpg.vector2pixel(nc)
        uv = (pixels.u, pixels.v)
        # rectify
        pr = gpg.rectify_point(uv)
        # recompute normalized coordinates
        t = Pixel(pr[0], pr[1])
        v = gpg.pixel2vector(t)
        pixels_normalized.append(v)

    s2 = Segment(color=s1.color, pixels_normalized=pixels_normalized)
    return s2


@dtu.contract(gpg=GroundProjectionGeometry, segment_list=SegmentList,
              returns=SegmentList)
def rectify_segments(gpg, segment_list):
    res = []

    for segment in segment_list.segments:
        s2 = rectify_segment(gpg, segment)
        res.append(s2)

    return SegmentList(segments=res)

