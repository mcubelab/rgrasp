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
#from PIL import Image
#from img_viewer import imgViewer

class GraspDataRecorder:#(threading.Thread):
  ##This class creates a package with the information gathered through the sensors
  def __init__(self):
    self.bridge = CvBridge()
    return

  def __reset_vars(self, grasp_id, directory, tote_num, frame_rate_ratio, image_size):
    #Dictionary with all the topics that we will need to subscribe (HARDCODED)
    self.topic_dict = {
                    'ws_0': {'topic':'ws_stream{}'.format(0), 'msg_format':Float64},
                    'ws_1': {'topic':'ws_stream{}'.format(1), 'msg_format':Float64},
                    'ws_2': {'topic':'ws_stream{}'.format(2), 'msg_format':Float64},
                    'gs_image': {'topic':'rpi/gelsight/raw_image', 'msg_format':Image},
                    'gs_deflection': {'topic':'rpi/gelsight/deflection', 'msg_format':Int32},
                    'gs_contactarea': {'topic':'rpi/gelsight/contactarea', 'msg_format':gelsight_contactarea},
                    'hand_commands': {'topic':'/hand_commands', 'msg_format':JointState},
                    'grasp_status': {'topic':'/grasp_status', 'msg_format':JointState},
                    'joint_states':{'topic':'/joint_states', 'msg_format':JointState},
                    'grasp_all_proposals': {'topic':'/grasp_all_proposals', 'msg_format':Float32MultiArray},
                    'grasp_proposal': {'topic':'/grasp_proposal', 'msg_format':Float32MultiArray},
                    'rgb_bin0': {'topic':'/arc_1/rgb_bin0', 'msg_format':Image},
                    'rgb_bin1': {'topic':'/arc_1/rgb_bin1', 'msg_format':Image},
                    'depth_bin0': {'topic':'/arc_1/depth_bin0', 'msg_format':Image},
                    'depth_bin1': {'topic':'/arc_1/depth_bin1', 'msg_format':Image},
                    'wsg_driver': {'topic':'/wsg_50_driver/status', 'msg_format':Status},
                    'exp_comments': {'topic':'/exp_comments', 'msg_format':String},
                    'impact_time': {'topic':'/impact_time', 'msg_format':String},
                    'objectType': {'topic':'/objectType', 'msg_format':String}
                    #'depth_arr_bin0': {'topic':'/arc_1/depth_arr_bin0', 'msg_format':Float32MultiArray},
                    #'depth_arr_bin1': {'topic':'/arc_1/depth_arr_bin1', 'msg_format':Float32MultiArray}
                    }

    #We delete the sensors we do not want to record
    if tote_num == 0:
        del self.topic_dict['ws_1']
        del self.topic_dict['ws_2']
        #del self.topic_dict['depth_arr_bin1']
        try:
            del self.topic_dict['rgb_bin1']
        except:
            pass
        try:
            del self.topic_dict['depth_bin1']
        except:
            pass
    elif tote_num == 1:
        del self.topic_dict['ws_0']
        del self.topic_dict['ws_2']
        #del self.topic_dict['depth_arr_bin0']
        try:
            del self.topic_dict['rgb_bin0']
        except:
            pass
        try:
            del self.topic_dict['depth_bin0']
        except:
            pass

    #We create the dictionary that will host all the info
    self.data_recorded = {'grasp_id':grasp_id, 'directory':directory, 'tote_num':tote_num, 'frame_rate_ratio':frame_rate_ratio, 'rgb_count':0, 'depth_count':0, 'image_size':image_size}
    for key in self.topic_dict:
        self.data_recorded[key] = {}

  def __get_image(self, data):
      arr = np.asarray(data)
      #~ min_val = np.amin(arr)
      #~ max_val = np.amax(arr)
      #~ mat = arr.reshape(480, 640)
      #return Image.fromarray(mat, "L")
      print mat
      iv = imgViewer(mat, min_val, max_val) 
      return mat
      

  def __callback(self, data, key):
    ##This function is called everytime a msg is published to one of our subscribed topics, it stores the data
    if self.topic_dict[key]['msg_format'] == Image:
        if key == 'rgb_bin0' or key=='rgb_bin1':
            self.data_recorded['rgb_count'] +=1
            if self.data_recorded['rgb_count']%self.data_recorded['frame_rate_ratio'] == 0:
                try:
                    cv2_img = self.bridge.imgmsg_to_cv2(data, 'bgr8') # Convert your ROS Image message to OpenCV2
                    if self.data_recorded['image_size'] != -1:
                        cv2_img = cv2.resize(cv2_img, self.data_recorded['image_size'])
                except CvBridgeError, e:
                    print(e)
                else:
                    self.data_recorded[key][rospy.get_time()] = cv2_img # Save your OpenCV2 image as a jpeg
        elif key == 'depth_bin0' or key=='depth_bin1':
            self.data_recorded['depth_count'] +=1
            if self.data_recorded['depth_count']%self.data_recorded['frame_rate_ratio'] == 0:
                try:
                    cv2_img = self.bridge.imgmsg_to_cv2(data, 'bgr16') # Convert your ROS Image message to OpenCV2
                    if self.data_recorded['image_size'] != -1:
                        cv2_img = cv2.resize(cv2_img, self.data_recorded['image_size'])
                except CvBridgeError, e:
                    print(e)
                else:
                    self.data_recorded[key][rospy.get_time()] = cv2_img # Save your OpenCV2 image as a jpeg
        else:
            try:
                cv2_img = self.bridge.imgmsg_to_cv2(data, 'bgr16') # Convert your ROS Image message to OpenCV2
                if self.data_recorded['image_size'] != -1:
                    cv2_img = cv2.resize(cv2_img, self.data_recorded['image_size'])
            except CvBridgeError, e:
                print(e)
            else:
                self.data_recorded[key][rospy.get_time()] = cv2_img # Save your OpenCV2 image as a jpeg
    #~ elif key=='depth_arr_bin0' or key=='depth_arr_bin1':
        #~ self.data_recorded['depth_count'] +=1
        #~ if self.data_recorded['depth_count']%self.data_recorded['frame_rate_ratio'] == 0:
            #~ image = self.__get_image(data.data)
            #~ self.data_recorded[key][rospy.get_time()] = image
    else:
      try:
        self.data_recorded[key][rospy.get_time()] = data.data
      except:
        self.data_recorded[key][rospy.get_time()] = data #Usually this exception is raised when data doesn't have a data field
    return

  def __plot_ws_evol(self):
    print 'Plotting result'
    try:
        plt.plot(list(self.data_recorded['ws_0'].values()))
    except:
        pass
    try:
        plt.plot(list(self.data_recorded['ws_1'].values()))
    except:
        pass
    try:
        plt.plot(list(self.data_recorded['ws_2'].values()))
    except:
        pass
    plt.show()

  def __save_raw_copy(self):
      print 'Saving raw copy'
      directory = self.data_recorded['directory'] +'/'+ str(self.data_recorded['grasp_id'])
      if not os.path.exists(directory): #If the directory does not exist, we create it
          os.makedirs(directory)
      if not os.path.exists(directory + '/images'): #If the directory does not exist, we create it
          os.makedirs(directory + '/images')
                    
      for key in self.data_recorded:
          if key in self.topic_dict:
              if self.topic_dict[key]['msg_format'] == Image:
                  i = 0
                  for item in self.data_recorded[key].values():
                    path = directory + '/images' + '/' + key + '_' + str(i) + '.png'
                    params = list()
                    params.append(16)
                    params.append(9)
                    cv2.imwrite(path, item, params)
                    i += 1
              #~ elif key=='depth_arr_bin0' or key=='depth_arr_bin1':
                  #~ i = 0
                  #~ for item in self.data_recorded[key].values():
                    #~ params = list()
                    #~ params.append(16)
                    #~ params.append(9)
                    #~ path = directory + '/images' + '/' + key + '_' + str(i) + '.png'
                    #~ cv2.imwrite(path, item, params)
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
      path = self.data_recorded['directory'] + '/' + str(self.data_recorded['grasp_id']) + '_info.p'
      pickle.dump(self.data_recorded, open(path, "wb"))
      print 'Dictionary saved'


  def start_recording(self, grasp_id, directory, tote_num=1, frame_rate_ratio=10, image_size=-1):
    print '################## RECORDING_NOW ############################'

    self.__reset_vars(grasp_id=grasp_id, directory=directory, tote_num=tote_num, frame_rate_ratio=frame_rate_ratio, image_size=image_size)

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

    del self.data_recorded['rgb_count']
    del self.data_recorded['depth_count']

    for elem in self.data_recorded:
        try:
            num_elem = len(self.data_recorded[elem].values())
        except:
            num_elem = 0
        print str(elem) + ': ' + str(num_elem)

    #Saving info package
    if not os.path.exists(self.data_recorded['directory']): #If the directory does not exist, we create one
      os.makedirs(directory)

    #We sort the dictionary
    for key in self.topic_dict:
         self.data_recorded[key] = collections.OrderedDict(sorted(self.data_recorded[key].items()))

    if save_dict:
        saving_dict = Process(target=self.__save_dict)
        saving_dict.start()

    if save_raw_copy:
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
