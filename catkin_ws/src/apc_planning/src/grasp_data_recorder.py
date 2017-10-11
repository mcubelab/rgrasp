import rospy, signal, sys, time, os, cv2
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import String
from pr_msgs.msg import gelsight_contactarea #~proper format: from <package> import <file>
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import numpy as np



class grasp_data_collector:
  ##This class creates a package with the information gathered through the sensors
  def __init__(self):
    self.ws_lectures = []
    
    self.bridge = CvBridge()
    self.gs_lectures_image = []
    self.gs_lectures_deflection = []
    self.gs_lectures_contact_area = []
    return
    
  def __ws_callback_0(self, data):
    #Reads and stores the messages published on the ws_stream
    print data.data
    self.ws_lectures.append(data.data)
  
  def __gs_callback_image(self, data):
    print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg
        self.gs_lectures_image.append(cv2_img)
    return
  
  def __gs_callback_deflection(self, data):
    #print("Received deflection value")
    #print data
    self.gs_lectures_deflection.append(data.data)
    return
  
  def __gs_callback_contactarea(self, data):
    #Reads and stores the messages published on the contactarea topic
    print data
    self.gs_lectures_contact_area.append(data)
    return
  
  #~ def __create_dict(grasp_id):
    
    
  
  def __plot_ws_evol(self):
    print 'Plotting result'
    plt.plot(self.ws_lectures)
    plt.show()

  def start_recording(self):
    rospy.init_node('grasp_data_collector', anonymous=True)
    
    #Weight sensor
    rospy.Subscriber('ws_stream{}'.format(0), Float64, self.__ws_callback_0) #Tote 1
    #rospy.Subscriber('ws_stream{}'.format(1), Float64, self.callback_1) #Tote 0
    #rospy.Subscriber('ws_stream{}'.format(2), Float64, self.callback_2) #Tote 2
    
    #Gelsight data
    rospy.Subscriber('rpi/gelsight/raw_image', Image, self.__gs_callback_image) 
    rospy.Subscriber('rpi/gelsight/deflection', Int32, self.__gs_callback_deflection)
    rospy.Subscriber('rpi/gelsight/contactarea', gelsight_contactarea, self.__gs_callback_contactarea)
    
  def stop_recording(self, grasp_id, directory, plot_ws=False):
    #Saving info package
    directory = directory + str(grasp_id)
    if not os.path.exists(directory): #If the directory does not exist, we create one
      os.makedirs(directory)
      print 'Directory created!'
    else:
      print 'Grasp id already existed! Data will be replaced'
    
    #Saving ws values
    path = directory + '/ws_values.txt'
    ws_file = open(path, 'w')
    for item in self.ws_lectures:
        ws_file.write("%s\n" % item)
    print 'Weight evolution saved'
    
    #Saving GS raw images
    i = 0
    for item in self.gs_lectures_image:
      path = directory + '/gs_image_' + str(i) + '.jpeg'
      cv2.imwrite(path, item)
      i += 1
    
    #Saving GS deflection
    path = directory + '/gs_deflection.txt'
    ws_file = open(path, 'w')
    for item in self.gs_lectures_deflection:
        ws_file.write("%s\n" % item)
    print 'GelSight deflection evolution saved'
    
    
    if plot_ws:
      self.__plot_ws_evol()
    return





gdr = grasp_data_collector() #Handler instatiation

gdr.start_recording()
time.sleep(5)
gdr.stop_recording(grasp_id='grasp_0', directory='/home/mcube/rgraspdata/', plot_ws=True)
