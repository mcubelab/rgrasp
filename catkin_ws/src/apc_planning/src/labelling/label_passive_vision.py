#!/usr/bin/env python

import cv2
import os
import re
import numpy as np

# cameraid = sys.argv[1] #'612203002922'
read_dir = os.environ["ARCDATA_BASE"] + "/loopdata/passive_vision/"
save_dir = os.environ["ARCDATA_BASE"] + "/labeldata/suction/"
good_dir = os.environ["ARCDATA_BASE"] + "/labeldata"
image = []
mask = []
color = (0,255,0)
color2 = (0,0,255)
drawing = 0
radius = 5
erasing = False

def draw(pos, color):
    if erasing:
        cv2.circle(mask, pos, radius, (0, 0, 0), -1)
    else:
        cv2.circle(mask, pos, radius, color, -1)

def click(event, x, y, flags, param):
    global drawing
    pos = (x, y)
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = 1
        draw(pos, color)
    elif event == cv2.EVENT_MBUTTONDOWN:
        drawing = 2
        draw(pos, color2)
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing == 1:
            draw(pos, color)
        elif drawing == 2:
            draw(pos, color2)
    elif event == cv2.EVENT_LBUTTONUP or event == cv2.EVENT_MBUTTONUP:
        drawing = 0

import signal
signal.signal(signal.SIGINT, signal.default_int_handler)
try:
    print 'Press e to switch drawing/erasing'
    print 'Press b for bigger brush and s for smaller'
    print 'Press n for saving and going to next image'
    print 'Press c to cancel edits and go to next image w/o saving'
    print 'Use left click to paint good points and middle click to paint bad ones'
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
                file_save = save_dir + x + '/' + f
                try:
                    image = cv2.imread(file_read)
                    if not os.path.exists(file_save):
                        mask = 0 * np.array(image)
                    else:
                        mask = cv2.imread(file_save)
                except:
                    continue
                cv2.namedWindow("image")
                cv2.setMouseCallback("image", click)
                cv2.imshow('image',image)
                key = ''
                while True:
                    cv2.imshow('image',cv2.add(image, mask))
                    # display the image and wait for a keypress
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("n"):
                        if not os.path.exists(save_dir + x):
                            os.makedirs(save_dir + x)
                        cv2.imwrite(save_dir + x + '/' + f, mask)
                        break
                    elif key == ord("c"):  # skip it
                        break
                    elif key == ord("e"):  # skip it
                        erasing = not erasing
                        print 'Set erasing to: ' + str(erasing)
                    elif key == ord("b"):  # skip it
                        radius += 1
                    elif key == ord("s"):  # skip it
                        radius = max(1, radius - 1)
                    elif key == ord('q'):
                        raise Exception('Done')
    print good_list
except Exception as e:
    print e.message
    cv2.destroyAllWindows()
