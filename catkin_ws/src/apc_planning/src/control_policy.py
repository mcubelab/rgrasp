#!/usr/bin/env python

import rospy, sys, os, tf, cv2
mcube_learning_path = os.environ['HOME'] + '/mcube_learning'
sys.path.append(mcube_learning_path)

from models.models import resnet_w_dense_layers, image_combined
from helper.image_helper import convert_world2image, convert_image2world, translate_image, crop_gelsight, back_substraction, preprocess_image
from helper.helper import load_file
from grasping17 import check_collision
from cv_bridge import CvBridge, CvBridgeError
# from PIL import Image
from helper.visualize_CAM import plot_CAM
import numpy as np
import scipy
import sensor_msgs
import pdb
import ik
from matplotlib import pyplot as plt
from std_msgs.msg import Float32MultiArray, String


def predict_success(model, img):
    img = np.expand_dims(img, axis=0)
    pred = model.predict(img)
    return np.squeeze(pred, axis=0)

def predict_successes(model, images):
    pred = model.predict(images)
    return pred

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

        self.action_dict_pub=rospy.Publisher('/action_dict',String,queue_size = 10, latch=True)
        self.best_action_dict_pub=rospy.Publisher('/best_action_dict',String,queue_size = 10, latch=True)




    def control_policy(self, back_img_list, binId=0):
        #load data
        background_images = back_img_list
        #capture image
        image_list = self.capture_images()
        #get current robot pose
        tcp_pose = ik.helper.get_tcp_pose(self.listener, tcp_offset = rospy.get_param("/gripper/spatula_tip_to_tcp_dist"))
        #perform background substraction
        image_sub_list=[]
        image_sub_list.append(back_substraction(image_list[0], background_images[0]))
        image_sub_list.append(back_substraction(image_list[1], background_images[1]))
        #generate new iamges
        self.action_dict = self.generate_new_images(image_sub_list, tcp_pose, binId)
        self.best_action_dict = self.select_best_action()

        # TODO: Save this 2 things

        return self.best_action_dict

    # def back_sub(self, image_list, back_list):
    #     for i in range(len(image_list)):
    #         image_list[i] = back_substraction(image_list[i], back_list[i])
    #     return image_list

    def capture_images(self):
        #capture images
        image_ros_list = []
        image_list = []
        for topic in self.image_topic_list:
            if rospy.get_param('have_robot'):
                image_ros = rospy.wait_for_message(topic, sensor_msgs.msg.Image)
                image_list.append(self.bridge.imgmsg_to_cv2(image_ros, 'rgb8'))
            else:
                image_path = '/media/mcube/data/Dropbox (MIT)/images/gelsight_fingerprint.png'
                image_list.append(cv2.imread(image_path, 1))
        return image_list

    def generate_new_images(self, back_image_list, tcp_pose, binId, visualize_score_map = True):
        out_dict = {}
        out_dict['images'] = []
        out_dict['images2'] = []
        out_dict['delta_pos'] = []
        #search actions through y and z grid (in world frame)
        y_range = np.linspace(-0.025, 0.025, 5)
        z_range = np.linspace(-0.01, 0.01, 5)
        self.score_map = np.zeros([len(y_range), len(z_range)])
        for it_y, y in enumerate(y_range):
            for it_z,z in enumerate(z_range):
                delta_pos = np.array([0,y,z])
                is_collision = check_collision(tcp_pose, delta_pos, self.listener, self.br, binId)
                # print is_collision
                #x in pixel frame -> y in hand frame
                #y in pixel frame -> -z in hand frame
                pos_pixel = convert_world2image(np.array([y,-z]))
                #resize image
                # img0 =  Image.fromarray(back_image_list[0])
                # img1 =  Image.fromarray(back_image_list[1])
                # img0 = img0.resize((224, 224))
                # img1 = img1.resize((224, 224))
                img0=scipy.misc.imresize(back_image_list[0], (224,224,3))
                img1=scipy.misc.imresize(back_image_list[1], (224,224,3))
                #2. crop and Resize
                img0 = scipy.misc.imresize(crop_gelsight(img0, bottom_edge = 40, top_edge = 0, left_edge = 15, right_edge = 18), (224,224,3))
                img1 = scipy.misc.imresize(crop_gelsight(img1,bottom_edge = 25, top_edge = 10, left_edge = 37, right_edge = 25), (224,224,3))
                #3. translate
                img0 = translate_image(img0, pos_pixel[0], pos_pixel[1])
                img1 = translate_image(img1, pos_pixel[0], pos_pixel[1])
                #4. preprocess
                img0 =preprocess_image(img0)
                img1 =preprocess_image(img1)
                #output images
                out_dict['images'].append(img0)
                out_dict['images2'].append(img1)
                out_dict['delta_pos'].append(-delta_pos)
                self.score_map[it_y][it_z] = self.model.predict([np.expand_dims(img0, axis=0), np.expand_dims(img1, axis=0)])[0][1]
        # if visualize_score_map:
        #     plt.pcolor(self.score_map, extent=[y_range[0], y_range[-1], z_range[0], z_range[-1]])
        #     plt.colorbar()
        #     plt.show()
        return out_dict

    def select_best_action(self):
        list_images = [np.array(self.action_dict['images']), np.array(self.action_dict['images2'])]
        predictions = predict_successes(self.model, list_images)
        self.action_dict['predictions'] = predictions
        best_index = np.argmax(predictions[:,1])
        out_dict = {}
        out_dict['image'] = self.action_dict['images'][best_index]
        out_dict['image2'] = self.action_dict['images2'][best_index]
        out_dict['delta_pos'] = self.action_dict['delta_pos'][best_index]
        out_dict['prediction'] = self.action_dict['predictions'][best_index]
        return out_dict

    def visualize_actions(self, with_CAM = True):
        num_img = 5
        for counter, image in enumerate(self.action_dict['images']):
            if np.mod(counter,5) == 0:
                plt.show()
                f, ax = plt.subplots(num_img, 1)
            combined_image = np.concatenate((self.action_dict['images'][counter], self.action_dict['images2'][counter]), axis=1)
            # ax[1].imshow(self.action_dict['images2'][counter], 'gray')
            ax[num_img-1-np.mod(counter,5)].set_title('Success: {}'.format(self.action_dict['predictions'][counter][1]))
            # ax[1].set_title('Success: {}'.format(self.action_dict['predictions'][counter][1]))
            # plt.xticks([])
            # plt.yticks([])
            if with_CAM:
                conv_layer = -1
                softmax_layer = -1 #Last layer
                desired_class=1
                conv_layers = [-7, -8]
                img = [np.expand_dims(self.action_dict['images'][counter], axis=0), np.expand_dims(self.action_dict['images2'][counter], axis=0)]
                CAM = plot_CAM(img, self.model, conv_layers, softmax_layer, desired_class, show_plot=False)
                CAM0 = np.zeros((CAM[0][0].shape[0], CAM[0][0].shape[1], 3))
                CAM0[:, :, 0] = CAM[0][0]
                CAM0[:, :, 1] = CAM[0][0]
                CAM0[:, :, 2] = CAM[0][0]
                CAM1 = np.zeros((CAM[0][1].shape[0], CAM[0][1].shape[1], 3))
                CAM1[:, :, 0] = CAM[0][1]
                CAM1[:, :, 1] = CAM[0][1]
                CAM1[:, :, 2] = CAM[0][1]
                combined_CAM = np.concatenate((CAM0, CAM1), axis=1)*0.5+combined_image*0.5
                combined_image = np.concatenate((combined_image, combined_CAM), axis=1)
            ax[num_img-1-np.mod(counter,5)].imshow(combined_image,  'gray')
        plt.close('all')
        return

    def visualize_best_action(self, with_CAM = True):
        f, ax = plt.subplots(1, 2)
        ax[0].imshow(self.best_action_dict['image'], 'gray')
        ax[1].imshow(self.best_action_dict['image2'], 'gray')
        ax[0].set_title('Success: {} delta_pos:{}'.format(self.best_action_dict['prediction'][1], self.best_action_dict['delta_pos']))
        # ax[1].set_title('Success: {}'.format(self.action_dict['predictions'][counter][1]))
        # plt.xticks([])
        # plt.yticks([])
        plt.show()
        plt.close('all')
        if with_CAM:
            conv_layer = -1
            softmax_layer = -1 #Last layer
            desired_class=1
            conv_layers = [-7, -8]
            img = [np.expand_dims(self.best_action_dict['image'], axis=0), np.expand_dims(self.best_action_dict['image2'], axis=0)]
            CAM = plot_CAM(img, self.model, conv_layers, softmax_layer, desired_class)
        return

# To test the function
if __name__=='__main__':
    rospy.init_node('control_policy', anonymous=True)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rospy.sleep(0.3)
    print 'start'
    #initialize model
    model_gelsight = resnet_w_dense_layers(layers_size=[], is_train = True)
    model_rgb = resnet_w_dense_layers(layers_size=[], is_train = True)
    for layer in model_rgb.layers:
        layer.name = layer.name + '_2'
    model = image_combined([model_gelsight, model_rgb], layers_size = [], activation_type = ['relu'], num_classes=2, is_train = True)

    topic_list = ["rpi/gelsight/flip_raw_image",  "rpi/gelsight/flip_raw_image2"]
    cp = controlPolicy(model, topic_list, listener, br)
    cp.control_policy()
    # cp.visualize_actions()
    print 'done'
    pdb.set_trace()
