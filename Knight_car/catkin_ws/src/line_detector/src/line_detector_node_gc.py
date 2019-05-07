#!/usr/bin/env python
from anti_instagram.AntiInstagram import *
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import (AntiInstagramTransform, BoolStamped, Segment,
    SegmentList, Vector2D)
from duckietown_utils.instantiate_utils import instantiate
from duckietown_utils.jpg import image_cv_from_jpg
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage, Image
from visualization_msgs.msg import Marker
from line_detector.line_detector_plot import *
from line_detector.timekeeper import TimeKeeper
import cv2
import numpy as np
import rospy
import threading
import time


class LineDetectorNode(object):
    def __init__(self):
        self.node_name = "LineDetectorNode"

        self.bw_1 = 0
        self.bw_2 = 0

    
        # Thread lock 
        self.thread_lock = threading.Lock()
       
        # Constructor of line detector 
        self.bridge = CvBridge()

        self.active = True

        self.stats = Stats()

        # Only be verbose every 10 cycles
        self.intermittent_interval = 100
        self.intermittent_counter = 0

        # color correction
        self.ai = AntiInstagram()

        # these will be added if it becomes verbose
        self.pub_edge = None
        self.pub_colorSegment = None

        self.detector = None
        self.verbose = None
        self.updateParams(None)
            
        # Publishers
        self.pub_lines = rospy.Publisher("~segment_list", SegmentList, queue_size=1)
        self.pub_image = rospy.Publisher("~image_with_lines", Image, queue_size=1)
       
        # Subscribers
        self.sub_image = rospy.Subscriber("~image", CompressedImage, self.cbImage, queue_size=1)
        self.sub_transform = rospy.Subscriber("~transform", AntiInstagramTransform, self.cbTransform, queue_size=1)
        self.sub_switch = rospy.Subscriber("~switch", BoolStamped, self.cbSwitch, queue_size=1)

        rospy.loginfo("[%s] Initialized (verbose = %s)." %(self.node_name, self.verbose))

        rospy.Timer(rospy.Duration.from_sec(2.0), self.updateParams)


    def updateParams(self, _event):
        old_verbose = self.verbose
        self.verbose = rospy.get_param('~verbose', True)
        # self.loginfo('verbose = %r' % self.verbose)
        if self.verbose != old_verbose:
            self.loginfo('Verbose is now %r' % self.verbose)

        self.image_size = rospy.get_param('~img_size')
        self.top_cutoff = rospy.get_param('~top_cutoff')

        if self.detector is None:
            c = rospy.get_param('~detector')
            assert isinstance(c, list) and len(c) == 2, c
        
#         if str(self.detector_config) != str(c):
            self.loginfo('new detector config: %s' % str(c))

            self.detector = instantiate(c[0], c[1])
#             self.detector_config = c

        if self.verbose and self.pub_edge is None:
            self.pub_edge = rospy.Publisher("~edge", Image, queue_size=1)
            self.pub_colorSegment = rospy.Publisher("~colorSegment", Image, queue_size=1)


    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data

    def cbImage(self, image_msg):
        self.stats.received()

        if not self.active:
            return 
        # Start a daemon thread to process the image
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()
        # Returns rightaway

    def cbTransform(self, transform_msg):
        self.ai.shift = transform_msg.s[0:3]
        self.ai.scale = transform_msg.s[3:6]

        self.loginfo("AntiInstagram transform received")

    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))

    def intermittent_log_now(self):
        return self.intermittent_counter % self.intermittent_interval == 1
    
    def intermittent_log(self, s):
        if not self.intermittent_log_now():
            return
        self.loginfo('%3d:%s' % (self.intermittent_counter, s))

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            self.stats.skipped()
            # Return immediately if the thread is locked
            return

        try:
            self.processImage_(image_msg)
        finally:
            # Release the thread lock
            self.thread_lock.release()

    def processImage_(self, image_msg):

        self.stats.processed()

        if self.intermittent_log_now():
            self.intermittent_log(self.stats.info())
            self.stats.reset()

        tk = TimeKeeper(image_msg)
        
        self.intermittent_counter += 1

        # Decode from compressed image with OpenCV
        try:
            image_cv = image_cv_from_jpg(image_msg.data)
        except ValueError as e:
            self.loginfo('Could not decode image: %s' % e)
            return

        tk.completed('decoded')

        # Resize and crop image
        hei_original, wid_original = image_cv.shape[0:2]

        if self.image_size[0] != hei_original or self.image_size[1] != wid_original:
            # image_cv = cv2.GaussianBlur(image_cv, (5,5), 2)
            image_cv = cv2.resize(image_cv, (self.image_size[1], self.image_size[0]),
                                   interpolation=cv2.INTER_NEAREST)
        image_cv = image_cv[self.top_cutoff:,:,:]

        tk.completed('resized')

        # apply color correction: AntiInstagram
        image_cv_corr = self.ai.applyTransform(image_cv)
        image_cv_corr = cv2.convertScaleAbs(image_cv_corr)

        tk.completed('corrected')

        # Set the image to be detected
        self.detector.setImage(image_cv_corr)

        # Detect lines and normals

        gray = cv2.cvtColor(image_cv_corr,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 200, apertureSize = 3)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, np.empty(1), 3, 1)

        hsv_yellow1 = (25,50,50)
        hsv_yellow2 = (45,255,255)

        hsv_blue1 = (100,90,80)
        hsv_blue2 = (150,255,155)

        #change color space to HSV
        hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)

        #find the color
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3, 3))

        yellow = cv2.inRange(hsv, hsv_yellow1, hsv_yellow2)
        yellow = cv2.dilate(yellow, kernel)
        self.bw_1=yellow

        blue = cv2.inRange(hsv, hsv_blue1, hsv_blue2)
        blue = cv2.dilate(blue, kernel) 
        self.bw_2=blue


        edge_color_yellow = cv2.bitwise_and(yellow, edges)
        edge_color_blue   = cv2.bitwise_and(blue, edges)

        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 10, np.empty(1), 3, 1)
        lines_yellow = cv2.HoughLinesP(edge_color_yellow, 1, np.pi/180, 10, np.empty(1), 3, 1)
        lines_blue = cv2.HoughLinesP(edge_color_blue, 1, np.pi/180, 10, np.empty(1), 3, 1)


        if lines_yellow is not None:
            lines_yellow = np.array(lines_yellow[0])
            print "found yellow lines"
    
        else:
            lines_yellow = []
            print "no yellow lines"
    
        if lines_blue is not None:
            lines_blue = np.array(lines_blue[0])
            print "found blue lines"

        else:
            lines_blue= []
            print "no blue lines"


        arr_cutoff = np.array((0, 40, 0, 40))
        arr_ratio = np.array((1./120, 1./160, 1./120, 1./160))

        lines_1 = lines_yellow
        lines_2 = lines_blue

        normals = []
        centers = []
        if len(lines_2)>0:
            #find the normalized coordinates
            lines_normalized = ((lines_1 + arr_cutoff) * arr_ratio)

            #find the unit vector
            length = np.sum((lines_1[:, 0:2] -lines_1[:, 2:4])**2, axis=1, keepdims=True)**0.5
            dx = 1.* (lines_1[:,3:4]-lines_1[:,1:2])/length
            dy = 1.* (lines_1[:,0:1]-lines_1[:,2:3])/length

            #find the center point
            centers = np.hstack([(lines_1[:,0:1]+lines_1[:,2:3])/2, (lines_1[:,1:2]+lines_1[:,3:4])/2])

       
            #find the vectors' direction
            x3 = (centers[:,0:1] - 4.*dx).astype('int')
            x3[x3<0]=0
            x3[x3>=160]=160-1

            y3 = (centers[:,1:2] - 4.*dy).astype('int')
            y3[y3<0]=0
            y3[y3>=120]=120-1

            x4 = (centers[:,0:1] + 4.*dx).astype('int')
            x4[x4<0]=0
            x4[x4>=160]=160-1

            y4 = (centers[:,1:2] + 4.*dy).astype('int')
            y4[y4<0]=0
            y4[y4>=120]=120-1
    
            flag_signs = (np.logical_and(self.bw_2[y3,x3]>0, self.bw_2[y4,x4]>0)).astype('int')*2-1
            normals = np.hstack([dx, dy]) * flag_signs
    
    
            flag = ((lines_1[:,2]-lines_1[:,0])*normals[:,1] - (lines_1[:,3]-lines_1[:,1])*normals[:,0])>0
            for i in range(len(lines_1)):
                if flag[i]:
                    x1,y1,x2,y2 = lines_1[i, :]
                    lines_1[i, :] = [x2,y2,x1,y1]


        tk.completed('detected')
     
        # SegmentList constructor
        segmentList = SegmentList()
        segmentList.header.stamp = image_msg.header.stamp
        
        # Convert to normalized pixel coordinates, and add segments to segmentList
        arr_cutoff = np.array((0, self.top_cutoff, 0, self.top_cutoff))
        arr_ratio = np.array((1./self.image_size[1], 1./self.image_size[0], 1./self.image_size[1], 1./self.image_size[0]))
        if len(lines_2) > 0:
            lines_normalized_blue = ((lines_2 + arr_cutoff) * arr_ratio)
            segmentList.segments.extend(self.toSegmentMsg(lines_normalized, normals,centers, 0))

        self.intermittent_log('# segments: white %3d ' % (len(lines_2)))
        
        tk.completed('prepared')

        # Publish segmentList
        self.pub_lines.publish(segmentList)
        tk.completed('--pub_lines--')

        # VISUALIZATION only below
        
        if self.verbose:

            # Draw lines and normals
            image_with_lines = np.copy(image_cv_corr)
            if len(lines_1)>0:
                for x1,y1,x2,y2 in lines_1:    
                    cx = int(x1+x2)/2
                    cy = int(y1+y2)/2
                    if cx >160:
                        cx = 160-1
                    elif cx<0:
                        cx=0
                    if cy >120:
                        cy = 120-1
                    elif cy<0:
                        cy=0
                    if(self.bw_2[cy,cx-1]==255 and self.bw_1[cy,cx+1]==255):
                        cv2.line(image_with_lines, (x1,y1), (x2,y2), (255,255,255))
                        cv2.circle(image_with_lines, (x1,y1), 1, (0,255,0)) #green circle
                        cv2.circle(image_with_lines, (x2,y2), 1, (255,0,0)) #red circle
                    if(self.bw_2[cy,cx+1]==255 and self.bw_1[cy,cx-1]==255):
                        cv2.line(image_with_lines, (x1,y1), (x2,y2), (255,255,255))
                        cv2.circle(image_with_lines, (x1,y1), 1, (0,255,0)) #green circle
                        cv2.circle(image_with_lines, (x2,y2), 1, (255,0,0)) #red circle
            #drawLines(image_with_lines, (lines_2), (255, 255, 255))
            #drawLines(image_with_lines, yellow.lines, (255, 0, 0))
            #drawLines(image_with_lines, red.lines, (0, 255, 0))

            tk.completed('drawn')

            # Publish the frame with lines
            image_msg_out = self.bridge.cv2_to_imgmsg(image_with_lines, "bgr8")
            image_msg_out.header.stamp = image_msg.header.stamp
            self.pub_image.publish(image_msg_out)

            tk.completed('pub_image')


#         if self.verbose:
            #colorSegment = color_segment(self.bw_1, self.bw_2) 
            #edge_msg_out = self.bridge.cv2_to_imgmsg(self.detector.edges, "mono8")
            #colorSegment_msg_out = self.bridge.cv2_to_imgmsg(colorSegment, "bgr8")
            #self.pub_edge.publish(edge_msg_out)
            #self.pub_colorSegment.publish(colorSegment_msg_out)

            tk.completed('pub_edge/pub_segment')

        self.intermittent_log(tk.getall())


    def onShutdown(self):
        self.loginfo("Shutdown.")
            
    def toSegmentMsg(self,  lines, normals,centers, color):
        segmentMsgList = []
        for x1,y1,x2,y2,norm_x,norm_y in np.hstack((lines,normals)):
            segment = Segment()
            segment.color = color
            segment.pixels_normalized[0].x = x1
            segment.pixels_normalized[0].y = y1
            segment.pixels_normalized[1].x = x2
            segment.pixels_normalized[1].y = y2
            segment.normal.x = norm_x
            segment.normal.y = norm_y
             
                #if  self.bw_2[y4,x4]>0 :
                 #   segment = Segment()
                  #  segment.color = color
                   # segment.pixels_normalized[0].x = x1
                    #segment.pixels_normalized[0].y = y1
                    #segment.pixels_normalized[1].x = x2
                    #segment.pixels_normalized[1].y = y2
                    #segment.normal.x = norm_x
                    #segment.normal.y = norm_y
            segmentMsgList.append(segment)
        return segmentMsgList

class Stats():
    def __init__(self):
        self.nresets = 0
        self.reset()

    def reset(self):
        self.nresets += 1
        self.t0 = time.time()
        self.nreceived = 0
        self.nskipped = 0
        self.nprocessed = 0

    def received(self):
        if self.nreceived == 0 and self.nresets == 1:
            rospy.loginfo('line_detector_node received first image.')
        self.nreceived += 1

    def skipped(self):
        self.nskipped += 1

    def processed(self):
        if self.nprocessed == 0 and self.nresets == 1:
            rospy.loginfo('line_detector_node processing first image.')

        self.nprocessed += 1

    def info(self):
        delta = time.time() - self.t0

        if self.nreceived:
            skipped_perc = (100.0 * self.nskipped / self.nreceived)
        else:
            skipped_perc = 0

        def fps(x):
            return '%.1f fps' % (x / delta)

        m = ('In the last %.1f s: received %d (%s) processed %d (%s) skipped %d (%s) (%1.f%%)' %
             (delta, self.nreceived, fps(self.nreceived),
              self.nprocessed, fps(self.nprocessed),
              self.nskipped, fps(self.nskipped), skipped_perc))
        return m





if __name__ == '__main__': 
    rospy.init_node('line_detector',anonymous=False)
    line_detector_node = LineDetectorNode()
    rospy.on_shutdown(line_detector_node.onShutdown)
    rospy.spin()



