#!/usr/bin/env python

#NOTE: this node might not be needed if the monodepth ROS repo just works

from __future__ import absolute_import, division, print_function


import os
import sys
import glob
import argparse
import numpy as np
import PIL.Image as pil
import matplotlib as mpl
import matplotlib.cm as cm

import torch
from torchvision import transforms, datasets

import monodepth2.networks
from monodepth2.layers import disp_to_depth
from monodepth2.utils import download_model_if_doesnt_exist

import rospy
from sensor_msgs.msg import CompressedImage


DEFAULT_MODEL_NAME = "mono+stereo_640x192"


class Monodepth2Node:

    def __init__(self, model_name, no_cuda):

        # Setup execution env
        if torch.cuda.is_available() and not no_cuda:
            self._device = torch.device("cuda")
        else:
            self._device = torch.device("cpu")

        # Get model
        download_model_if_doesnt_exist(model_name)
        dir_path = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(dir_path,"monodepth2","models",model_name)
        encoder_path = os.path.join(model_path, "encoder.pth")
        depth_decoder_path = os.path.join(model_path, "depth.pth")

        # Load encoder
        self._encoder = networks.ResnetEncoder(18, False)
        loaded_dict_enc = torch.load(encoder_path, map_location=self._device)

        # extract the height and width of image that this model was trained with
        self._feed_height = loaded_dict_enc['height']
        self._feed_width = loaded_dict_enc['width']
        filtered_dict_enc = {k: v for k, v in loaded_dict_enc.items() if k in self._encoder.state_dict()}
        self._encoder.load_state_dict(filtered_dict_enc)
        self._encoder.to(self._device)
        self._encoder.eval()

        # Load decoder
        self._depth_decoder = networks.DepthDecoder(num_ch_enc=self._encoder.num_ch_enc, scales=range(4))

        loaded_dict = torch.load(depth_decoder_path, map_location=self._device)
        self._depth_decoder.load_state_dict(loaded_dict)

        self._depth_decoder.to(self._device)
        self._depth_decoder.eval()

        # ROS image subscriber and publiser
        self._img_pub = rospy.Publisher('monodepth2')


    def _img_cb(self, msg):
        with torch.no_grad():

            #TODO unpack to image
            #input_image = pil.open(image_path).convert('RGB')
            # TODO input image

            original_width, original_height = input_image.size
            input_image = input_image.resize((self._feed_width, self._feed_height), pil.LANCZOS)
            input_image = transforms.ToTensor()(input_image).unsqueeze(0)

            # Predict
            input_image = input_image.to(self._device)
            features = self._encoder(input_image)
            outputs = self._depth_decoder(features)

            disp = outputs[("disp", 0)]
            disp_resized = torch.nn.functional.interpolate(
                disp, (original_height, original_width), mode="bilinear", align_corners=False)

            scaled_disp, _ = disp_to_depth(disp, 0.1, 100)

            #TODO pack and output to ROS


if __name__ == "__main__":
    rospy.init_node('monodepth2_node')

    model_name = rospy.get_param('model_name',DEFAULT_MODEL_NAME)
    no_cuda = rospy.get_param('no_cuda',True)

    node = Monodepth2Node(model_name,no_cuda)

    rospy.spin()
