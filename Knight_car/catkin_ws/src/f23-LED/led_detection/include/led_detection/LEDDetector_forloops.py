from api import LEDDetector
from duckietown_msgs.msg import Vector2D, LEDDetection, LEDDetectionArray
from led_detection import logger
from math import floor, ceil
import numpy as np
import rospy
import time

# plotting 
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.patches import Rectangle

# image filters
from scipy.ndimage.filters import maximum_filter
from scipy.ndimage.morphology import generate_binary_structure, binary_erosion

# fft
import scipy.fftpack

__all__ = ['LEDDetector']

class LEDDetector_forloops():
    """ The LEDDetector class """

    def __init__(self, ploteverything_=False, verbose_=False, plotfinal_=False):
        self.ploteverything = ploteverything_
        self.verbose = verbose_
        self.plotfinal = plotfinal_
        pass

    # ~~~~~~~~~~~~~~~~~~~ Downsample ~~~~~~~~~~~~~~~~~~~~~~~~

    def downsample(self, channel, cell_width=20, cell_height=20):
        W = channel.shape[2]
        H = channel.shape[1]
        cell_intensity_data = []
        for x in range(0, W, cell_width):
            for y in range(0, H, cell_height):
                #logger.info('Looking at partition %s %s' % (x, y))
                # look at same partition across multiple images
                average_intensities = []
                for image in channel[:15]: # TODO hardcoded value
                    intensities = []
                    # loop through pixels in each partition
                    for x_coord in range(x, x+cell_width):
                        for y_coord in range(y,y+cell_height):
                            if x_coord >= W or y_coord >= H:
                                continue
                            pixel = image[y_coord][x_coord]
                            # a simple approach from http://stackoverflow.com/questions/596216/formula-to-determine-brightness-of-rgb-color
                            # could also use matplotlib's rgb_to_hsv?
                            # TODO: check if pixel order is RGB or BGR (assuming RGB)
                            #intensity = 0.299*pixel[0] + 0.587*pixel[1] + 0.114*pixel[2]
                            #intensity = pixel[0] # red channel
                            #intensity = pixel[1] # green channel
                            intensity = pixel # blue channel
                            intensities.append(intensity)


                    average_intensities.append(np.amax(intensities))
                cell_intensity_data.append(average_intensities)
        out = np.array(cell_intensity_data)
        print(1.0*W/cell_width)
        print(1.0*H/cell_height)
        print(1.0*W/cell_width*1.0*H/cell_height)
        # HACK1 
        out = np.reshape(out, [ceil(1.0*W/cell_width), ceil(1.0*H/cell_height), out.shape[1]]) # TODO change the code above later
        #HACK2
        out = np.swapaxes(out, 0,2)
        print(out.shape)
        #plt.imshow(out[:,:,0])
        return (out, [0,0])

    # ~~~~~~~~~~~~~~~~~~~ Find local maxima ~~~~~~~~~~~~~~~~~~~~

    def detect_peaks(self, image):
        """
        Takes an image and detect the peaks usingthe local maximum filter.
        Returns a boolean mask of the peaks (i.e. 1 when
        the pixel's value is the neighborhood maximum, 0 otherwise)
        """
        neighborhood = generate_binary_structure(2,2)
        local_max = maximum_filter(image, 5)==image
        background = (image==0)
        eroded_background = binary_erosion(background, structure=neighborhood, border_value=1)
        detected_peaks = local_max - eroded_background
        return detected_peaks

    # ~~~~~~~~~~~~~~~~~~~ Select candidates ~~~~~~~~~~~~~~~~~~~~

    def get_candidate_cells(self, cell_values, threshold):
        variance = np.var(cell_values, axis=0)
        peaks_mask = self.detect_peaks(variance)
        threshold_mask = variance>threshold
        if(self.ploteverything):
            plt.figure()
            plt.imshow(variance,cmap=cm.gray, interpolation="nearest")
            plt.title('Variance map')
            plt.figure()
            plt.imshow(peaks_mask*threshold_mask)
            plt.title('Peaks')
            plt.show()
    
        return peaks_mask*threshold_mask

    # ~~~~~~~~~~~~~~~~~~~ Detect LEDs ~~~~~~~~~~~~~~~~~~~~~~~~~~~

    def detect_led(self,
                   images,
                   mask,
                   frequencies_to_detect,
                   min_distance_between_LEDs_pixels):

        assert len(images.shape) == 1
        n = images.shape[0]
        if n == 0:
            raise ValueError('No images provided')

        timestamps = images['timestamp']
        rgb = images['rgb']

        rgb0 = rgb[0]
        if not mask.shape == rgb0.shape:
            raise ValueError('Invalid mask')

        if not isinstance(frequencies_to_detect, list):
            raise ValueError(frequencies_to_detect)

        if not min_distance_between_LEDs_pixels > 0:
            raise ValueError(min_distance_between_LEDs_pixels)

        channel = images['rgb'][:,:,:,0] # just using first channel

        cell_width = 15
        cell_height = 15

        (cell_vals, crop_offset) = self.downsample(channel, cell_width, cell_height)

        candidates_mask = self.get_candidate_cells(cell_vals, 100)
        candidate_cells = [(i,j) for (i,j) in np.ndindex(candidates_mask.shape) if candidates_mask[i,j]]

        # Create result object
        result = LEDDetectionArray()

        # Detect frequencies and discard non-periodic signals
        # ts_tolerance = 0.2 # unnecessary
        f_tolerance = 0.25
        min_num_periods = 3

        for (i,j) in candidate_cells:
            signal = cell_vals[:,i,j]
            signal = signal-np.mean(signal)

            zero_crossings = np.where(np.diff(np.sign(signal)))[0]
            zero_crossings_t = timestamps[zero_crossings]
            led_img_coords = Vector2D((0.5+j)*cell_width+crop_offset[1], (0.5+i)*cell_height+crop_offset[0])       
            diffs = [b-a for a, b in zip(zero_crossings_t, zero_crossings_t[1:])]
            
            if(self.verbose):
                logger.info('Coords: %s, %s'% (led_img_coords.x,led_img_coords.y))
                logger.info('Zero crossings: %s'%zero_crossings_t)
                logger.info('Diffs: %s'%diffs)
                logger.info('Zero-crossing measured freq %s'% (0.5/np.mean(diffs)))

            if(len(zero_crossings)<min_num_periods):
                if(self.verbose):
                    logger.info('Not an LED, discarded\n')
                continue

            # Frequency estimation based on zero crossings - quite bad
            #for f in frequencies_to_detect:
            #    if(all(d-ts_tolerance <= 0.5/f <= d+ts_tolerance for d in diffs)):
            #        if(self.verbose):
            #            logger.info('Confirmed LED with frequency %s\n'%f)
                   # recover coordinates of centroid
            #        result.detections.append(LEDDetection(timestamps[0], timestamps[-1],
            #        led_img_coords, f, '', -1)) # -1...confidence not implemented
            #        break

            # Frequency estimation based on FFT
            T = 1.0/30 # TODO expecting 30 fps, but RESAMPLE to be sure
            f = np.linspace(0.0, 1.0/(2.0*T), n/2)
            signal_f = scipy.fftpack.fft(signal)
            y_f =  2.0/n * np.abs(signal_f[:n/2])
            fft_peak_freq = 1.0*np.argmax(y_f)/T/n
            if(self.verbose):
                logger.info('FFT peak frequency: %s'% fft_peak_freq)

            # Bin frequency into the ones to detect
            freq = [x for x in frequencies_to_detect if abs(x-fft_peak_freq)<f_tolerance]
            if(freq):
                result.detections.append(LEDDetection(rospy.Time.from_sec(timestamps[0]),
                rospy.Time.from_sec(timestamps[-1]), led_img_coords, freq[0], '', -1)) # -1...confidence not implemented
                if(self.verbose):
                    logger.info('LED confirmed, frequency: %s'% freq)
            else:
                logger.info('Could not associate frequency, discarding')

            print(signal.shape)
            print(timestamps[:15].shape)
            # Plot all signals and FFTs
            if(self.ploteverything):
                fig, ax1 = plt.subplots()
                ax1.plot(timestamps[:15], signal)
                fig, ax2 = plt.subplots()
                ax2.plot(f[:15],y_f)
                plt.show()

        plt.imshow(rgb0)
        ax = plt.gca()

        font = {'family': 'serif',
                'color':  'red',
                'weight': 'bold',
                'size': 16,
                }

        # Plot all results
        if(self.plotfinal):
            for r in result.detections:
                pos = r.pixels_normalized
                ax.add_patch(Rectangle((pos.x-0.5*cell_width, pos.y-0.5*cell_height), cell_width, cell_height, edgecolor="red", linewidth=3, facecolor="none"))
                plt.text(pos.x-0.5*cell_width, pos.y-cell_height, str(r.frequency), fontdict=font)

            plt.show()

        return result


