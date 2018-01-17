#!/usr/bin/env python

import rospy, sys, os, tf, cv2
mcube_learning_path = os.environ['HOME'] + '/mcube_learning'
sys.path.append(mcube_learning_path)

from models.models import resnet_w_dense_layers, image_combined
from helper.image_helper import convert_world2image, convert_image2world, translate_image
from helper.helper import load_file
from grasping17 import check_collision
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import sensor_msgs
import pdb
import ik
from matplotlib import pyplot as plt


# def capture_image(topic_name):
#     img = rospy.wait_for_message(topic_name, sensor_msgs.msg.Image)
#     return img
#
# def predict_success(model, img):
#     img = np.expand_dims(img, axis=0)
#     pred = model.predict(img)
#     return np.squeeze(pred, axis=0)
#
# def predict_successes(model, images):
#     pred = model.predict(images)
#     return pred
#
#
# def select_best_action(model, action_dict):
#     predictions = predict_successes(model, np.array(action_dict['images']))
#     best_index = np.argmax(predictions[:,1])
#     out_dict = {}
#     out_dict['image'] = action_dict['images'][best_index]
#     out_dict['delta_pos'] = action_dict['delta_pos'][best_index]
#     return out_dict


class controlPolicy():
    '''
    This class preprocesses the dictionary "dataset_dict" in the ds class and creates the processed dictionary "timed_dataset_dict"
    '''
    def __init__(self, model, image_topic_list, listener, br):
        self.model = model
        self.image_topic_list = image_topic_list
        self.listener = listener
        self.br = br
        self.bridge = CvBridge()


    def control_policy(self, binId=0):
        #load data
        #capture image
        image_list = self.capture_images()
        #get current robot pose
        tcp_pose = ik.helper.get_tcp_pose(self.listener, tcp_offset = rospy.get_param("/gripper/spatula_tip_to_tcp_dist"))
        #generate new images
        action_dict = self.generate_new_images(image_list, tcp_pose, binId)
        best_action_dict = select_best_action(model, action_dict)

    def capture_images(self):
        #capture images
        image_ros_list = []
        image_list = []
        for topic in self.image_topic_list:
            if rospy.get_param('have_robot'):
                image_ros = rospy.wait_for_message(topic)
                image_list.append(self.bridge.imgmsg_to_cv2(image_ros, 'rgb8'))
            else:
                image_path = os.environ['CODE_BASE'] + '/docs/images/gelsight_fingerprint.png'
                image_list.append(cv2.imread(image_path, 1))
        return image_list

    def generate_new_images(self, image_list, tcp_pose, binId):
        out_dict = {}
        out_dict['images'] = []
        out_dict['images2'] = []
        out_dict['delta_pos'] = []
        #search actions through y and z grid (in world frame)
        y_range = np.linspace(-0.025, 0.025, 2)
        z_range = np.linspace(-0.01, 0.01, 2)

        for y in y_range:
            for z in z_range:
                delta_pos = np.array([0,y,z])
                is_collision = check_collision(tcp_pose, delta_pos, self.listener, self.br, binId)
                # print is_collision
                #y in world frame -> x in pixel frame
                #z in world frame -> y in pixel frame
                pos_pixel = convert_world2image(np.array([y,z]))

                #translate image
                out_dict['images'].append(translate_image(image_list[0], pos_pixel[0], pos_pixel[1]))
                out_dict['images2'].append(translate_image(image_list[1], pos_pixel[0], pos_pixel[1]))
                out_dict['delta_pos'].append(delta_pos)

        return out_dict

# To test the function
if __name__=='__main__':
    rospy.init_node('control_policy', anonymous=True)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rospy.sleep(0.3)
    #initialize model
    model_single = resnet_w_dense_layers(layers_size=[], activation_type=['relu'], num_classes=2, is_train = False)
    model_single2 = vgg16_w_dense_layers(layers_size=[], activation_type=['relu'], num_classes=2, is_train = False)
    model = image_combined([model_single, model_single2]], layers_size = [32], activation_type = ['relu'], num_classes = None, is_train = False)
    topic_list = ["rpi/gelsight/flip_raw_image",  "rpi/gelsight/flip_raw_image2"]
    cp = controlPolicy(model, topic_list, listener, br)
    # control_policy(listener, br)
    pdb.set_trace()
