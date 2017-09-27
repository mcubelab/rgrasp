#!/usr/bin/env python

import cv2
import os
import re
import numpy as np
import json
from math import cos, sin, pi

# cameraid = sys.argv[1] #'612203002922'
read_dir = os.environ["ARCDATA_BASE"] + "/heightmap/"
save_dir = os.environ["ARCDATA_BASE"] + "/labeldata/grasping/"
good_dir = os.environ["ARCDATA_BASE"] + "/labeldata"
image = []
mask = []
mask2 = []
mask_marker = []
grasp_points_good = []
grasp_buffer_good = []
grasp_points_bad = []
grasp_buffer_bad = []
color = [(0,0,0),(0,255,0),(0,0,255)]
color_marker = (0, 255, 255)
drawing = 0
width = 10
min_width = 5
max_width = 57
angle = 0
angle_speed = 10.0
pre_pos = None

def reset_local():
    global mask2, grasp_buffer_good, grasp_buffer_bad
    print 'Discarding local changes'
    mask2 = mask.copy()
    grasp_buffer_good = []
    grasp_buffer_bad = []

def update_local():
    global mask, grasp_points_good, grasp_points_bad
    print 'Updating local changes'
    mask = mask2.copy()
    grasp_points_good += grasp_buffer_good
    grasp_points_bad += grasp_buffer_bad
    reset_local()

def draw(pos):
    hx = 0.5 * width * cos(angle)
    hy = 0.5 * -width * sin(angle)
    new_pos1 = (int(pos[0] - hx), int(pos[1] - hy))
    new_pos2 = (int(pos[0] + hx), int(pos[1] + hy))
    if drawing:
        cv2.line(mask2, new_pos1, new_pos2, color[drawing], 2)
        if drawing == 1:
            grasp_buffer_good.append([[x for x in new_pos1], [x for x in new_pos2]])
        else:
            grasp_buffer_bad.append([[x for x in new_pos1], [x for x in new_pos2]])
    else:
        global pre_pos
        if pre_pos is not None:
            cv2.line(mask_marker, pre_pos[0], pre_pos[1], color[0], 2)
            cv2.circle(mask_marker, pre_pos[0], 5, color[0], -1)
        cv2.circle(mask_marker, new_pos1, 5, (255, 255, 255), -1)
        cv2.line(mask_marker, new_pos1, new_pos2, color_marker, 2)
        pre_pos = [new_pos1, new_pos2]

def click(event, x, y, flags, param):
    global drawing
    pos = (x, y)
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = 1
        draw(pos)
    elif event == cv2.EVENT_MBUTTONDOWN:
        drawing = 2
        draw(pos)
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == 1:
            draw(pos)
        elif drawing == 2:
            draw(pos)
        elif drawing == 0:
            draw(pos)
    elif event == cv2.EVENT_LBUTTONUP or event == cv2.EVENT_MBUTTONUP:
        drawing = 0

import signal
signal.signal(signal.SIGINT, signal.default_int_handler)
try:
    print 'Press d to discard your local changes'
    print 'Press f to save your local changes'

    print 'Press n for saving and going to next image'
    print 'Press c to cancel edits and go to next image w/o saving'

    print 'Press b for bigger width'
    print 'Press s for smaller width'
    print 'Press j to turn clockwise'
    print 'Press k to turn counterclockwise'

    print 'Use left click to paint good points and right click to paint bad ones'
    print 'Press q to exit the program'
    good_list = []
    bad_list = []
    with open(good_dir + '/out', 'r') as f:
        lines = f.readlines()
    for line in lines:
        good_list.append(line[0:-1])
    try:
        with open(good_dir + '/ignore.out', 'r') as f:
            lines = f.readlines()
        for line in lines:
            bad_list.append(line[0:-1])
    except Exception as e:
        print e.message
        pass
    for x in next(os.walk(read_dir))[1]:
        if x in bad_list:
            continue
        if not x in good_list:
            continue
        print x
        for f in next(os.walk(read_dir + x + '/'))[2]:
            if re.match(r'^passive-vision-input\.0.*color.png$', f):
                print f
                file_read = read_dir + x + '/' + f
                file_save = (save_dir + x + '/' + f)[0:-3]
                file_save_png = file_save + 'png'
                file_save_good = file_save + 'good.json'
                file_save_bad = file_save + 'bad.json'
                try:
                    image = cv2.imread(file_read)
                    if not os.path.exists(file_save_png):
                        mask = 0 * np.array(image)
                        grasp_points_good = []
                        grasp_points_bad = []
                    else:
                        mask = cv2.imread(file_save_png)
                        with open(file_save_good, 'r') as infile:
                            grasp_points_good = json.load(infile)
                        with open(file_save_bad, 'r') as infile:
                            grasp_points_bad = json.load(infile)
                    mask_marker = 0 * np.array(image)
                    reset_local()
                except Exception as e:
                    print e.message
                    continue
                cv2.namedWindow("image")
                cv2.setMouseCallback("image", click)
                cv2.imshow('image',image)
                key = ''
                while True:
                    cv2.imshow('image', cv2.add(cv2.add(image, mask2), mask_marker))
                    # display the image and wait for a keypress
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("n"):
                        update_local()
                        if not os.path.exists(save_dir + x):
                            os.makedirs(save_dir + x)
                        with open(file_save_good, 'w') as outfile:
                            json.dump(grasp_points_good, outfile)
                        with open(file_save_bad, 'w') as outfile:
                            json.dump(grasp_points_bad, outfile)
                        cv2.imwrite(file_save_png, mask)
                        break
                    elif key == ord("c"):
                        break
                    elif key == ord("b"):
                        width = min(max_width, width + 1)
                    elif key == ord("s"):
                        width = max(min_width, width - 1)
                    elif key == ord("f"):
                        update_local()
                    elif key == ord("d"):
                        reset_local()
                    elif key == ord("j"):
                        angle = (angle + angle_speed / 360.0) % (2 * pi)
                    elif key == ord("k"):
                        angle = (angle - angle_speed / 360.0) % (2 * pi)
                    elif key == ord('q'):
                        raise Exception('Done')
    # print good_list
except Exception as e:
    print e.message
    cv2.destroyAllWindows()
