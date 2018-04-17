#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This script visualize the semantic segmentation of ENet.
"""
import os
import numpy as np
from argparse import ArgumentParser
from os.path import join
import argparse
import sys
caffe_root = '/home/robert/ENet/caffe-enet/'  # Change this to the absolute directory to ENet Caffe
sys.path.insert(0, caffe_root + 'python')
import caffe
sys.path.append('/usr/local/lib/python2.7/site-packages')
import cv2
import time

__author__ = 'Timo Sämann'
__university__ = 'Aschaffenburg University of Applied Sciences'
__email__ = 'Timo.Saemann@gmx.de'
__data__ = '24th May, 2017'


def make_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, required=True, help='.prototxt file for inference')
    parser.add_argument('--weights', type=str, required=True, help='.caffemodel file')
    parser.add_argument('--colours', type=str, required=True, help='label colours')
    parser.add_argument('--input_image', type=str, required=True, help='input image path')
    parser.add_argument('--out_dir', type=str, default=None, help='output directory in which the segmented images '
                                                                   'should be stored')
    parser.add_argument('--gpu', type=str, default='0', help='0: gpu mode active, else gpu mode inactive')

    return parser


if __name__ == '__main__':
    parser1 = make_parser()
    args = parser1.parse_args()
    caffe.set_mode_gpu()
    # if args.gpu == 0:
    #     caffe.set_mode_gpu()
    #     print "GPU"
    #     time.sleep(5)
    # else:
    #     caffe.set_mode_cpu()
    #     print "CPU"
    #     time.sleep(5)

    net = caffe.Net(args.model, args.weights, caffe.TEST)

    input_shape = net.blobs['data'].data.shape
    print input_shape
    output_shape = net.blobs['deconv6_0_0'].data.shape

    label_colours = cv2.imread(args.colours, 1).astype(np.uint8)


    cap = cv2.VideoCapture("/home/robert/Videos/test3.webm")
    # cap = cv2.VideoCapture("/home/robert/Videos/cartcam.MP4")
    # cap = cv2.VideoCapture("/dev/video0")

    videoOut = cv2.VideoWriter('roads.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 30, (600 * 2, 360 * 2), True)


    rval = True

    frameNum = 0

    videoStart = time.time()

    while rval:
        frameStart = time.time()
        start = time.time()

        rval, frame = cap.read()#.astype(np.float32)
        if rval == False:
            break

        end = time.time()
        print '%30s' % 'Grabbed camera frame in ', str((end - start)*1000), 'ms'

        start = time.time()
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.resize(frame, (input_shape[3],input_shape[2]))
        input_image = frame.transpose((2,0,1))
        #input_image = input_image[(2,1,0),:,:] # May be required, if you do not open your data with opencv
        input_image = np.asarray([input_image])
        end = time.time()
        print '%30s' % 'Resized image in ', str((end - start)*1000), 'ms'
        # cv2.imshow("Input", frame)

        start = time.time()
        out = net.forward_all(data=input_image)
        #out = net.forward_all(**{net.inputs[0]: input_image})
        end = time.time()
        print '%30s' % 'Executed ENet in ', str((end - start)*1000), 'ms'

        start = time.time()
        prediction = net.blobs['deconv6_0_0'].data[0].argmax(axis=0)
        prediction = np.squeeze(prediction)
        prediction = np.resize(prediction, (3, input_shape[2], input_shape[3]))
        prediction = prediction.transpose(1, 2, 0).astype(np.uint8)
        
        prediction_rgb = np.zeros(prediction.shape, dtype=np.uint8)
        label_colours_bgr = label_colours[..., ::-1]
        cv2.LUT(prediction, label_colours_bgr, prediction_rgb)
        end = time.time()
        print '%30s' % 'Processed results in ', str((end - start)*1000), 'ms\n'
        print '%30s' % 'Processed frame in ', str((end - frameStart)*1000), 'ms\n'
        print '%30s' % 'FPS:', str(1/(end-frameStart)), '\n'

	overlay = cv2.addWeighted(frame, 1, prediction_rgb, 0.7, 0)

        combined = np.zeros((360*2, 600 * 2, 3), dtype="uint8") # Height * Width * Channels
        combined[0:360, 0:600] = frame
        combined[0:360, 600:1200] = prediction_rgb
	combined[360:360*2, 300:900] = overlay
        videoOut.write(combined)
#        cv2.imshow("Combined", combined)
        # cv2.imshow("ENet", prediction_rgb)
        key = cv2.waitKey(1)

    # if args.out_dir is not None:
    #     input_path_ext = args.input_image.split(".")[-1]
    #     input_image_name = args.input_image.split("/")[-1:][0].replace('.' + input_path_ext, '')
    #     out_path_im = args.out_dir + input_image_name + '_enet' + '.' + input_path_ext
    #     out_path_gt = args.out_dir + input_image_name + '_enet_gt' + '.' + input_path_ext

    #     cv2.imwrite(out_path_im, prediction_rgb)
    #     # cv2.imwrite(out_path_gt, prediction) #  label images, where each pixel has an ID that represents the class
print '%30s' % 'Finished video in ', str((time.time() - videoStart)*1000), 'ms\n'





