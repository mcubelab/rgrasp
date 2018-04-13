#!/usr/bin/env python

import os, sys
import numpy as np
import rospy
import sensor_msgs.msg
import cv2
from cv_bridge import CvBridge
sys.path.append(os.environ['CODE_BASE']+'/catkin_ws/src/weight_sensor/src')
sys.path.append(os.environ['HOME'] + '/mcube_learning')
from helper.image_helper import get_center_of_mass, crop_contact
from scipy import ndimage
import gripper
import rospy
import copy

def flip_image(data, topic):
    #flip image using opencv
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    flip_cv_image = cv2.flip(cv_image,1)
    flip_cv_image = cv2.flip(flip_cv_image,0)
    outImage = flip_cv_image
    if rospy.get_param('/is_high_viz', True):
        back_cv_image = cv2.imread('/home/mcube/background_0.png', 1)
        patch_cv_image, initial_img, contact_img = crop_contact(back_cv_image, flip_cv_image, gel_id = 1, is_zeros=True)
        COM_pos = get_center_of_mass(contact_img)
        COM_pos = np.nan_to_num(COM_pos)
        print(COM_pos)
        
        #publish flipped image
        # Convert uint8 to float
        foreground = initial_img
        #foreground = foreground[:-140,55:-70]  #im_crop_grasp = img_grasp[0:-120,135:-135]
        img = contact_img*255
        
        # threshold image
        ret, threshed_img = cv2.threshold(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY),
                        127, 255, cv2.THRESH_BINARY)
        # find contours and get the external one
        image, contours, hier = cv2.findContours(threshed_img, cv2.RETR_TREE,
                        cv2.CHAIN_APPROX_SIMPLE)
         
        # with each contour, draw boundingRect in green
        # a minAreaRect in red and
        # a minEnclosingCircle in blue
        biggest_area = 0
        biggest_rect = None
        for c in contours:
            
            # get the min area rect
            rect = cv2.minAreaRect(c)
            area = float(rect[1][0])*float(rect[1][1])
            if biggest_area < area: 
                biggest_rect = rect
                biggest_area = area
        
        box = cv2.boxPoints(biggest_rect)
        # convert all coordinates floating point values to int
        box = np.int0(box)
        # draw a red 'nghien' rectangle
        cv2.drawContours(img, [box], 0, (0, 0, 255), 2)
        cv2.rectangle(img, (int(biggest_rect[0][0]-10),int(biggest_rect[0][1]-10)),
                    (int(biggest_rect[0][0]+10), int(biggest_rect[0][1]+10)),(0, 255, 0), -1)
        cv2.rectangle(img, (int(COM_pos[1]-10),int(COM_pos[0]-10)),
                    (int(COM_pos[1]+10), int(COM_pos[0]+10)),(0, 0, 255), -1)
        outimg = cv2.drawContours(img, contours, -1, (255, 255, 0), 2)
        
        # Normalize the alpha mask to keep intensity between 0 and 1
        alpha = 0.4
        
        #img = cv2.resize(img, (foreground.shape[1], foreground.shape[0]))
        # Add the masked foreground and background.
        outImage = cv2.addWeighted(foreground,alpha,img,1-alpha,0)
    

    flip_image_pub = rospy.Publisher(topic,sensor_msgs.msg.Image, queue_size = 10)
    flip_image_pub.publish(bridge.cv2_to_imgmsg(outImage, "bgr8"))

def flip_image2(data, topic):
    #flip image using opencv
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # flip_cv_image = cv2.flip(cv_image,1)
    flip_cv_image = cv2.flip(cv_image,0)
    outImage = flip_cv_image
    if rospy.get_param('/is_high_viz', True):
        back_cv_image = cv2.imread('/home/mcube/background_1.png', 1)
        patch_cv_image, initial_img, contact_img = crop_contact(back_cv_image, flip_cv_image, gel_id = 2, is_zeros=True)
        COM_pos = get_center_of_mass(contact_img)
        COM_pos = np.nan_to_num(COM_pos)
        print(COM_pos)
        
        #publish flipped image
        # Convert uint8 to float
        foreground = initial_img
        #foreground = foreground[:-140,55:-70]  #im_crop_grasp = img_grasp[0:-120,135:-135]
        img = contact_img*255
        
        # threshold image
        ret, threshed_img = cv2.threshold(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY),
                        127, 255, cv2.THRESH_BINARY)
        # find contours and get the external one
        image, contours, hier = cv2.findContours(threshed_img, cv2.RETR_TREE,
                        cv2.CHAIN_APPROX_SIMPLE)
         
        # with each contour, draw boundingRect in green
        # a minAreaRect in red and
        # a minEnclosingCircle in blue
        biggest_area = 0
        biggest_rect = None 
        for c in contours:
            
            # get the min area rect
            rect = cv2.minAreaRect(c)
            area = float(rect[1][0])*float(rect[1][1])
            if biggest_area < area: 
                biggest_rect = rect
                biggest_area = area
        if biggest_rect:
            box = cv2.boxPoints(biggest_rect)
            # convert all coordinates floating point values to int
            box = np.int0(box)
            # draw a red 'nghien' rectangle
            cv2.drawContours(img, [box], 0, (0, 0, 255), 2)
            cv2.rectangle(img, (int(biggest_rect[0][0]-10),int(biggest_rect[0][1]-10)),
                        (int(biggest_rect[0][0]+10), int(biggest_rect[0][1]+10)),(0, 255, 0), -1)
            cv2.rectangle(img, (int(COM_pos[1]-10),int(COM_pos[0]-10)),
                        (int(COM_pos[1]+10), int(COM_pos[0]+10)),(0, 0, 255), -1)
            
            outimg = cv2.drawContours(img, contours, -1, (255, 255, 0), 2)
            # Normalize the alpha mask to keep intensity between 0 and 1
            alpha = 0.4
            
            #img = cv2.resize(img, (foreground.shape[1], foreground.shape[0]))
            # Add the masked foreground and background.
            outImage = cv2.addWeighted(foreground,alpha,img,1-alpha,0)
            
            #rotation angle in degree
            rotated = cv2.pyrDown(cv2.imread('/home/mcube/vertical_nut.png', cv2.IMREAD_UNCHANGED))
            rows = rotated.shape[1]
            cols = rotated.shape[1]
            
            if (gripper.getGripperopening() > 0.03):
                if biggest_rect[1][0] > biggest_rect[1][1]:
                    M = cv2.getRotationMatrix2D((cols/2,rows/2),rect[2]+90,1)
                else:
                    M = cv2.getRotationMatrix2D((cols/2,rows/2),rect[2],1)
                rotated = cv2.warpAffine(rotated,M,(cols,rows))
            else:
                rotated = cv2.imread('/home/mcube/hor2_nut.png', cv2.IMREAD_UNCHANGED)
            
            rotated = cv2.resize(rotated, (outImage.shape[1], outImage.shape[0]))
            outImage = np.concatenate([outImage, rotated], axis=1)
            
        else:
            outImage = foreground

    flip_image_pub = rospy.Publisher(topic,sensor_msgs.msg.Image, queue_size = 10)
    flip_image_pub.publish(bridge.cv2_to_imgmsg(outImage, "bgr8"))

def flip_image2_basic(data, topic):
    #flip image using opencv
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # flip_cv_image = cv2.flip(cv_image,1)
    flip_cv_image = cv2.flip(cv_image,0)
    outImage = flip_cv_image
    flip_image_pub = rospy.Publisher(topic,sensor_msgs.msg.Image, queue_size = 10)
    flip_image_pub.publish(bridge.cv2_to_imgmsg(outImage, "bgr8"))

def flip_image_basic(data, topic):
    #flip image using opencv
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    flip_cv_image = cv2.flip(cv_image,1)
    flip_cv_image = cv2.flip(flip_cv_image,0)
    outImage = flip_cv_image
    flip_image_pub = rospy.Publisher(topic,sensor_msgs.msg.Image, queue_size = 10)
    flip_image_pub.publish(bridge.cv2_to_imgmsg(outImage, "bgr8"))

def listener():
    image_sub = rospy.Subscriber("rpi/gelsight/raw_image", sensor_msgs.msg.Image, flip_image, "rpi/gelsight/flip_raw_image")
    image_sub2 = rospy.Subscriber("rpi/gelsight/raw_image2", sensor_msgs.msg.Image, flip_image2, "rpi/gelsight/flip_raw_image2")
    image_sub3 = rospy.Subscriber("rpi/gelsight/raw_image", sensor_msgs.msg.Image, flip_image_basic, "rpi/gelsight/flip_raw_image_basic")
    image_sub4 = rospy.Subscriber("rpi/gelsight/raw_image2", sensor_msgs.msg.Image, flip_image2_basic, "rpi/gelsight/flip_raw_image2_basic")
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('gs_flip_publisher', anonymous=True)
    print 'start node'
    listener()
