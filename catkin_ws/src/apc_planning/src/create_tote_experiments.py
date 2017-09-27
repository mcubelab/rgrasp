import subprocess
import scipy.stats
import random as rand
import time, datetime
from tabulate import tabulate
#import gripper
#import suction
import json
import numpy as np
#from PIL import Image
import pylab
import Image
import matplotlib.cm as cm
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

if __name__ == '__main__':
    
    jsonfilename = '/home/mcube/arc/input/apc_stow_task_experiment.json'
    with open(jsonfilename) as data_file:
        DATA = json.load(data_file)
        
    toteObjects = DATA['tote_contents']
    toteObjects.sort()
    number_objects_experiment = 20
    rand.shuffle(toteObjects)
    toteObjects = toteObjects[0:number_objects_experiment]
    toteObjects.sort()
    print(toteObjects)

    #for obj in toteObjects:
        #img = Image.open('~/Desktop/%s.jpg' %(obj))
        #img.show()
    f = pylab.figure()
    
    for n, obj in enumerate(toteObjects): #fname in enumerate(('1.png', '2.png')):
        
        f.add_subplot(number_objects_experiment/5, 5, n+1)  # this line outputs images on top of each other
        img=mpimg.imread('/home/mcube/Desktop/objects_images_arc_2017/%s/%s_Top_01.png' %(obj, obj))
        pylab.imshow(img) #arr,cmap=cm.Greys_r)
        pylab.title(obj)
        plt.axis('off')
    
    pylab.show()
    pylab.show()
