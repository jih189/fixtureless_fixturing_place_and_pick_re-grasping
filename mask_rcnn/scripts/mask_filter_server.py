#!/usr/bin/env python3

import os
import sys
import rospy
from sensor_msgs.msg import Image
from object_segmentation.srv import Object_segmentation, Object_segmentationResponse
import numpy as np

import torch
import torchvision
import cv2
from torchvision.transforms import transforms as transforms

from train import get_model_instance_segmentation
from output_utils import draw_segmentation_map, get_outputs, get_mask_by_name



class object_segmentation_server:
    def __init__(self):
        self.server = rospy.Service("object_filter", Object_segmentation, self.image_filter)

        # initialize the model
        self.model = get_model_instance_segmentation(4)

        # set the computation device
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        # load the modle on to the computation device and set to eval mode
        self.model.to(self.device).eval()
        model_path = os.path.dirname(os.path.realpath(__file__)) + "/result/model_maybe.pth"
        checkpoint = torch.load(model_path, map_location='cpu')
        self.model.load_state_dict(checkpoint['model'])

        # transform to convert the image to tensor
        self.transform = transforms.Compose([
            transforms.ToTensor()
        ])
        print("Ready to run filter!")
        self.object_name = "book"
        rospy.spin()

    def imgmsg_to_cv2(self, img_msg):
        # if img_msg.encoding != "bgr8":
        #     rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
        dtype = np.dtype("uint8") # Hardcode to 8 bits...
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                        dtype=dtype, buffer=img_msg.data)
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()

        if img_msg.encoding == "rgb8":
            image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_BGR2RGB)

        return image_opencv

    def cv2_to_imgmsg(self, cv_image):
        img_msg = Image()
        img_msg.height = cv_image.shape[0]
        img_msg.width = cv_image.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.data = cv_image.tostring()
        img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
        return img_msg


    def image_filter(self, req):
        try:
            cv_image = self.imgmsg_to_cv2(req.Image)
        except Exception as e:
            print(e)

        orig_image = cv_image.copy()
        # transform the image
        cv_image = self.transform(cv_image.copy())
        # add a batch dimension
        cv_image = cv_image.unsqueeze(0).to(self.device)
        masks, boxes, labels = get_outputs(cv_image, self.model, 0.5)
        if masks.shape[-2:] == cv_image.shape[-2:]:
            cv_image = draw_segmentation_map(orig_image, masks, boxes, labels)
            object_mask = get_mask_by_name(masks, labels, self.object_name)
            cv_image[object_mask==False]=(0,0,0)
            cv_image[object_mask]=(255,255,255)

            res = Object_segmentationResponse()
            res.Result = self.cv2_to_imgmsg(cv_image)

        return res


if __name__ == '__main__':
    rospy.init_node('image_filter_server')
    oss = object_segmentation_server()