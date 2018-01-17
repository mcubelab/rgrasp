#!/usr/bin/env python

import rospy, sys, os, tf, cv2
mcube_learning_path = os.environ['HOME'] + '/mcube_learning'
sys.path.append(mcube_learning_path)

from models.models import resnet_w_dense_layers
from helper.image_helper import convert_world2image, convert_image2world, translate_image
from helper.helper import load_file
from grasping17 import check_collision

import numpy as np
import sensor_msgs
import pdb
import ik


# def capture_image(topic_name):
#     img = rospy.wait_for_message(topic_name, sensor_msgs.msg.Image)
#     return img
#
def predict_success(model, img):
    img = np.expand_dims(img, axis=0)
    pred = model.predict(img)
    return np.squeeze(pred, axis=0)

def predict_successes(model, images):
    pred = model.predict(images)
    return pred

def generate_new_images(img, tcp_pose, listener, br, binId=0):
    out_dict = {}
    out_dict['images'] = []
    out_dict['delta_pos'] = []
    #search actions through y and z grid (in world frame)
    y_range = np.linspace(-0.025, 0.025, 10)
    z_range = np.linspace(-0.01, 0.01, 10)

    for y in y_range:
        for z in z_range:
            delta_pos = np.array([0,y,z])
            is_collision = check_collision(tcp_pose, delta_pos, listener, br, binId=0)
            # print is_collision
            #y in world frame -> x in pixel frame
            #z in world frame -> y in pixel frame
            pos_pixel = convert_world2image(np.array([y,z]))

            #translate image
            out_dict['images'].append(translate_image(img, pos_pixel[0], pos_pixel[1]))
            out_dict['delta_pos'].append(delta_pos)

    return out_dict
#
# def get_best_action():
#     pass


# To test the function
if __name__=='__main__':
    rospy.init_node('control_policy', anonymous=True)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rospy.sleep(0.3)
    #initialize model
    model = resnet_w_dense_layers(layers_size=[], activation_type=['relu'], num_classes=2, is_train = True)
    #load data
    image_path = os.environ['CODE_BASE'] + '/docs/images/gelsight_fingerprint.png'
    #define image
    img = cv2.imread(image_path, 1)
    tcp_pose = ik.helper.get_tcp_pose(listener, tcp_offset = rospy.get_param("/gripper/spatula_tip_to_tcp_dist"))

    images = generate_new_images(img, tcp_pose, listener, br, binId=0)
    pdb.set_trace()
