#!/usr/bin/env python

import numpy as np 
import cv2 
import sys
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import time as timer
import rospkg as rp

def add_gaussian_shifts(depth, std=1/2.0):

    rows, cols = depth.shape 
    gaussian_shifts = np.random.normal(0, std, size=(rows, cols, 2))
    gaussian_shifts = gaussian_shifts.astype(np.float32)

    # creating evenly spaced coordinates  
    xx = np.linspace(0, cols-1, cols)
    yy = np.linspace(0, rows-1, rows)

    # get xpixels and ypixels 
    xp, yp = np.meshgrid(xx, yy)

    xp = xp.astype(np.float32)
    yp = yp.astype(np.float32)

    xp_interp = np.minimum(np.maximum(xp + gaussian_shifts[:, :, 0], 0.0), cols)
    yp_interp = np.minimum(np.maximum(yp + gaussian_shifts[:, :, 1], 0.0), rows)

    depth_interp = cv2.remap(depth, xp_interp, yp_interp, cv2.INTER_LINEAR)

    return depth_interp
    

def filterDisp(disp, dot_pattern_, invalid_disp_):

    size_filt_ = 9

    xx = np.linspace(0, size_filt_-1, size_filt_)
    yy = np.linspace(0, size_filt_-1, size_filt_)

    xf, yf = np.meshgrid(xx, yy)

    xf = xf - int(size_filt_ / 2.0)
    yf = yf - int(size_filt_ / 2.0)

    sqr_radius = (xf**2 + yf**2)
    vals = sqr_radius * 1.2**2 

    vals[vals==0] = 1 
    weights_ = 1 /vals  

    fill_weights = 1 / ( 1 + sqr_radius)
    fill_weights[sqr_radius > 9] = -1.0 

    disp_rows, disp_cols = disp.shape 
    dot_pattern_rows, dot_pattern_cols = dot_pattern_.shape

    lim_rows = np.minimum(disp_rows - size_filt_, dot_pattern_rows - size_filt_)
    lim_cols = np.minimum(disp_cols - size_filt_, dot_pattern_cols - size_filt_)

    center = int(size_filt_ / 2.0)

    window_inlier_distance_ = 0.1

    out_disp = np.ones_like(disp) * invalid_disp_

    interpolation_map = np.zeros_like(disp)

    # filterDisp_starttime = timer.time()

    for r in range(0, lim_rows):

        for c in range(0, lim_cols):

            if dot_pattern_[r+center, c+center] > 0:
                                
                # c and r are the top left corner 
                window  = disp[r:r+size_filt_, c:c+size_filt_] 
                dot_win = dot_pattern_[r:r+size_filt_, c:c+size_filt_] 
  
                valid_dots = dot_win[window < invalid_disp_]

                n_valids = np.sum(valid_dots) / 255.0 
                n_thresh = np.sum(dot_win) / 255.0 

                if n_valids > n_thresh / 1.2: 

                    mean = np.mean(window[window < invalid_disp_])

                    diffs = np.abs(window - mean)
                    diffs = np.multiply(diffs, weights_)

                    cur_valid_dots = np.multiply(np.where(window<invalid_disp_, dot_win, 0), 
                                                 np.where(diffs < window_inlier_distance_, 1, 0))

                    n_valids = np.sum(cur_valid_dots) / 255.0

                    if n_valids > n_thresh / 1.2: 
                    
                        accu = window[center, center] 

                        assert(accu < invalid_disp_)

                        out_disp[r+center, c + center] = round((accu)*8.0) / 8.0

                        interpolation_window = interpolation_map[r:r+size_filt_, c:c+size_filt_]
                        disp_data_window     = out_disp[r:r+size_filt_, c:c+size_filt_]

                        substitutes = np.where(interpolation_window < fill_weights, 1, 0)
                        interpolation_window[substitutes==1] = fill_weights[substitutes ==1 ]

                        disp_data_window[substitutes==1] = out_disp[r+center, c+center]
    # print "filter Disp time = ", timer.time() - filterDisp_starttime

    return out_disp

scale_factor  = 500     # converting depth from m to cm 
focal_length  = 480.0   # focal length of the camera used 
baseline_m    = 0.075   # baseline in m 
invalid_disp_ = 99999999.9


class noise_adder:

    def __init__(self, dot_pattern):
        # reading the image directly in gray with 0 as input 
        self.dot_pattern_ = dot_pattern
        # scale_percent = 200 # percent of original size
        # width = int(dot_pattern.shape[1] * scale_percent / 100)
        # height = int(dot_pattern.shape[0] * scale_percent / 100)
        # self.dot_pattern_ = cv2.resize(dot_pattern, (width, height), interpolation=cv2.INTER_AREA)
        self.image_pub = rospy.Publisher("/head_camera/noise_depth/image_raw", Image, queue_size=1)
        self.cam_info_pub = rospy.Publisher("/head_camera/noise_depth/camera_info", CameraInfo, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/head_camera/depth/image_raw",Image, self.callback, queue_size=1)
        self.cam_info_sub = rospy.Subscriber("/head_camera/depth/camera_info", CameraInfo, self.cam_info_callback, queue_size=1)
        self.camInfo = None

    def cam_info_callback(self, data):
        self.camInfo = data

    def callback(self, data):
        depth_image_raw = self.bridge.imgmsg_to_cv2(data, "passthrough").byteswap()
        # totaltimestart = timer.time()

        # print "max value = ", np.max(depth_image_raw), " min value = ", np.min(depth_image_raw)
        # cv2.imshow("test", depth_image_raw)
        # cv2.waitKey(1)

        h, w = depth_image_raw.shape

        # Our depth images were scaled by 5000 to store in png format so dividing to get 
        # depth in meters 
        depth = depth_image_raw.astype('float') / 5000.0

        depth_interp = add_gaussian_shifts(depth)

        disp_= focal_length * baseline_m / (depth_interp + 1e-10)
        depth_f = np.round(disp_ * 8.0)/8.0

        out_disp = filterDisp(depth_f, self.dot_pattern_, invalid_disp_)
        # out_disp = depth_f
        

        depth = focal_length * baseline_m / out_disp
        depth[out_disp == invalid_disp_] = 0 
        
        # The depth here needs to converted to cms so scale factor is introduced 
        # though often this can be tuned from [100, 200] to get the desired banding / quantisation effects
        with np.errstate(divide='ignore'):
            noisy_depth = (35130/np.round((35130/np.round(depth*scale_factor)) + np.random.normal(size=(h, w))*(1.0/6.0) + 0.5))/scale_factor 

        noisy_depth = noisy_depth * 5000.0 
        noisy_depth = noisy_depth.astype('uint16')

        # print "total time = ", timer.time() - totaltimestart

        # cv2.imshow("test", noisy_depth)
        # cv2.waitKey(1)

        try:
            output = self.bridge.cv2_to_imgmsg(noisy_depth, "passthrough")
            # output = self.bridge.cv2_to_imgmsg(depth_image_raw, "passthrough")
            output.header = data.header
            time = rospy.Time.now()
            output.header.stamp = time
            self.camInfo.header.stamp = time
            self.camInfo.header.seq = output.header.seq
            self.image_pub.publish(output)
            self.cam_info_pub.publish(self.camInfo)
        except CvBridgeError as e:
            print(e)

def main(args):
    rospy.init_node('depth_image_noise_adder', anonymous=True)
    na = noise_adder(cv2.imread(rp.RosPack().get_path('simkinect') + "/scripts/data/kinect-pattern_3x3.png", 0))
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
        main(sys.argv)