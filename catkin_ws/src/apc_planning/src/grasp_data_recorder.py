import rospy, signal, sys, time, os, cv2, pickle, collections
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from pr_msgs.msg import gelsight_contactarea #~proper format: from <package> import <file>
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from wsg_50_common.msg import Status
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt
import numpy as np
import threading
from multiprocessing import Process

class GraspDataRecorder:#(threading.Thread):
  ##This class creates a package with the information gathered through the sensors
  def __init__(self, grasp_id, directory):#, node):
    #threading.Thread.__init__(self)
    self.bridge = CvBridge()
    #self.node = node

    #Dictionary with all the topics that we will need to subscribe (HARDCODED)
    self.topic_dict = {
                    'ws_0': {'topic':'ws_stream{}'.format(1), 'msg_format':Float64},
                    'ws_1': {'topic':'ws_stream{}'.format(0), 'msg_format':Float64},
                    'ws_2': {'topic':'ws_stream{}'.format(2), 'msg_format':Float64},
                    'gs_image': {'topic':'rpi/gelsight/raw_image', 'msg_format':Image},
                    'gs_deflection': {'topic':'rpi/gelsight/deflection', 'msg_format':Int32},
                    'gs_contactarea': {'topic':'rpi/gelsight/contactarea', 'msg_format':gelsight_contactarea},
                    'hand_commands': {'topic':'/hand_commands', 'msg_format':JointState},
                    'grasp_status': {'topic':'/grasp_status', 'msg_format':JointState},
                    'joint_states':{'topic':'/joint_states', 'msg_format':JointState},
                    'grasp_all_proposals': {'topic':'/grasp_all_proposals', 'msg_format':Float32MultiArray},
                    'grasp_proposal': {'topic':'/grasp_proposal', 'msg_format':Float32MultiArray},
                    'camera_bin0': {'topic':'/arc_1/camera_bin0', 'msg_format':Image},
                    'camera_bin1': {'topic':'/arc_1/camera_bin1', 'msg_format':Image},
                    'camera_depth_bin0': {'topic':'/arc_1/camera_bin0/compressedDepth', 'msg_format':CompressedImage},
                    'camera_depth_bin1': {'topic':'/arc_1/camera_bin0/compressedDepth', 'msg_format':CompressedImage},
                    'wsg_driver': {'topic':'/wsg_50_driver/status', 'msg_format':Status}
                    }

    #We create the dictionary that will host all the info
    self.data_recorded = {'grasp_id': grasp_id, 'directory':directory}
    for key in self.topic_dict:
        self.data_recorded[key] = {}
    return

  def __callback(self, data, key):
    ##This function is called everytime a msg is published to one of our subscribed topics, it stores the data
    if self.topic_dict[key]['msg_format'] == Image:
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Convert your ROS Image message to OpenCV2
        except CvBridgeError, e:
            print(e)
        else:
            #print 'Image received!'
            self.data_recorded[key][rospy.get_time()] = cv2_img # Save your OpenCV2 image as a jpeg
    else:
      try:
        #print data.data
        self.data_recorded[key][rospy.get_time()] = data.data
      except:
        #print 'Non-standard msg received'
        self.data_recorded[key][rospy.get_time()] = data #Usually this exception is raised when data doesn't have a data field
    return

  def __plot_ws_evol(self):
    print 'Plotting result'
    #plt.plot(list(self.data_recorded['ws_0'].values()))
    plt.plot(list(self.data_recorded['ws_1'].values()))
    #plt.plot(list(self.data_recorded['ws_2'].values()))
    plt.show()

  def __save_raw_copy(self):
      print 'Saving raw copy'
      directory = self.data_recorded['directory'] +'/grasp_'+ str(self.data_recorded['grasp_id'])
      if not os.path.exists(directory): #If the directory does not exist, we create it
          os.makedirs(directory)
      
      for key in self.data_recorded:
          if key in self.topic_dict:
              if self.topic_dict[key]['msg_format'] == Image:
                  if not os.path.exists(directory + '/images'): #If the directory does not exist, we create it
                    os.makedirs(directory + '/images')
                  i = 0
                  for item in self.data_recorded[key].values():
                    path = directory + '/images' + '/' + key + '_' + str(i) + '.jpeg'
                    cv2.imwrite(path, item)
                    i += 1
              else:
                  path = directory + '/' + key + '.txt'
                  ws_file = open(path, 'w')
                  for elem in self.data_recorded[key]:
                      val = str(elem) + ' ' + str(self.data_recorded[key][elem]) #val = time + data
                      ws_file.write("%s\n" % val)
      print 'Raw copy saved'

  def __save_dict(self):
      print 'Saving dictionary'
      path = self.data_recorded['directory'] + '/grasp_' + str(self.data_recorded['grasp_id']) + '_info.p'
      pickle.dump(self.data_recorded, open(path, "wb"))
      print 'Dictionary saved'

  def start_recording(self):
    print '################## RECORDING_NOW ############################'
    try:
        #rospy.init_node('grasp_data_collector', anonymous=True) #We intialize the listener node
        pass
    except e:
        print e
    
    self.subscribers = {}
    for key in self.topic_dict: #We subscribe to every topic in the topic_dict
        topic = self.topic_dict[key]['topic']
        msg_format = self.topic_dict[key]['msg_format']
        self.subscribers[key] = rospy.Subscriber(topic, msg_format, self.__callback, key)

  def stop_recording(self, save_dict=True, save_raw_copy=False, plot_ws=False):
    print '################## STOPPING_RECORDING ############################'
    #We unregister from the topics
    for key in self.subscribers:
        self.subscribers[key].unregister()
    
    #Saving info package
    if not os.path.exists(self.data_recorded['directory']): #If the directory does not exist, we create one
      os.makedirs(directory)

    #We sort the dictionary
    for key in self.topic_dict:
         self.data_recorded[key] = collections.OrderedDict(sorted(self.data_recorded[key].items()))

    if save_dict:
        #processThread = threading.Thread(target=self.__save_dict)
        #processThread.start()
        saving_dict = Process(target=self.__save_dict)
        saving_dict.start()

    if save_raw_copy:
        #self.__save_raw_copy()
        #processThread = threading.Thread(target=self.__save_raw_copy)
        #processThread.start()
        saving_raw = Process(target=self.__save_raw_copy)
        saving_raw.start()
        
    if saving_dict and saving_raw:
        saving_dict.join()
        saving_raw.join()

    if plot_ws:
        self.__plot_ws_evol()
    return


#node = rospy.init_node('grasp_data_collector', anonymous=True) #We intialize the listener node
#gdr = GraspDataRecorder(grasp_id=0, directory='/home/mcube/rgraspdata')#, node=node) #Handler instatiation

#gdr.start_recording()
#time.sleep(5)
#gdr.stop_recording(save_dict=True, save_raw_copy=True, plot_ws=True)
#print gdr.data_recorded


