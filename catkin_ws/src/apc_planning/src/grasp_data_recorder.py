import rospy, signal, sys, time, os, cv2, pickle, collections
from std_msgs.msg import Int32, Float64, Float32MultiArray, String, Bool
from pr_msgs.msg import gelsight_contactarea
from sensor_msgs.msg import JointState, Image, CompressedImage
from wsg_50_common.msg import Status
from cv_bridge import CvBridge, CvBridgeError
from msg_to_dict import convert_ros_message_to_dictionary
import matplotlib.pyplot as plt
from multiprocessing import Process
import numpy as np
import pandas as pd
import thread

class GraspDataRecorder:#(threading.Thread):
  ##This class creates a package with the information gathered through the sensors
  def __init__(self, directory, exp_id=''):
    self.bridge = CvBridge()

    if exp_id == '':
        exp_id = 'EXP_' + str(rospy.get_time())
    self.exp_id = exp_id
    self.directory = directory + '/' + str(exp_id)

    #Directory creation
    if not os.path.exists(self.directory):
      os.makedirs(self.directory)

    self.experiment_info = []
    return

  def __reset_vars(self, action, action_id, tote_num, frame_rate_ratio, image_size):
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
                    'impact_time': {'topic':'/impact_time', 'msg_format':Bool},
                    'objectList': {'topic':'/objectList', 'msg_format':Float32MultiArray},
                    'objectType': {'topic':'/objectType', 'msg_format':String}
                    }

    #We delete the sensors we do not want to record
    if tote_num == 0:
        del self.topic_dict['ws_1']
        del self.topic_dict['ws_2']
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
        try:
            del self.topic_dict['rgb_bin0']
        except:
            pass
        try:
            del self.topic_dict['depth_bin0']
        except:
            pass

    #We create the dictionary that will host all the info
    self.data_recorded = {'exp_id': self.exp_id, 'action':action, 'action_id':action_id, 'tote_num':tote_num, 'frame_rate_ratio':frame_rate_ratio, 'rgb_count':0, 'depth_count':0, 'image_size':image_size, 'object_id':'', 'directory':self.directory}
    for key in self.topic_dict:
        self.data_recorded[key] = []

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
                    self.data_recorded[key].append((cv2_img, rospy.get_time()))
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
                    self.data_recorded[key].append((cv2_img, rospy.get_time()))
        else:
            try:
                cv2_img = self.bridge.imgmsg_to_cv2(data, 'bgr16') # Convert your ROS Image message to OpenCV2
                if self.data_recorded['image_size'] != -1:
                    cv2_img = cv2.resize(cv2_img, self.data_recorded['image_size'])
            except CvBridgeError, e:
                print(e)
            else:
                self.data_recorded[key].append((cv2_img, rospy.get_time()))
    else:
      try:
        self.data_recorded[key].append((data.data, rospy.get_time()))
      except:
        self.data_recorded[key].append((data, rospy.get_time())) #Usually this exception is raised when data doesn't have a data field
    return

  def __save_raw_copy(self):
      print '[RECORDER]: Saving raw copy'
      directory = self.data_recorded['directory'] +'/'+ str(self.data_recorded['action_id'])
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
              else:
                  path = directory + '/' + key + '.txt'
                  ws_file = open(path, 'w')
                  for elem in self.data_recorded[key]:
                      val = str(elem) + ' ' + str(self.data_recorded[key][elem]) #val = time + data
                      ws_file.write("%s\n" % val)
      print '[RECORDER]: Raw copy saved'

  def __save_action(self):
      print '[RECORDER]: Saving action data...'

      #Directory creation
      path = self.data_recorded['directory'] + '/' + str(self.data_recorded['action_id'])
      if not os.path.exists(path):
        os.makedirs(path)

      #We save all the readings from the sensors
      for key in self.topic_dict:
          values_filename = str(key) + '_values'
          timestamps_filename = str(key) + '_timestamps'
          try:
            values, timestamps = zip(*self.data_recorded[key])
            np.savez_compressed(path + '/' + values_filename, values)
            np.savez_compressed(path + '/' + timestamps_filename, timestamps)
          except ValueError:
            pass


      #pickle.dump(self.data_recorded, open(path, "wb"))
      print '[RECORDER]: Saving DONE'


  def __get_grasp_summary(self):
      #General info
      info = {
        'exp_id': self.data_recorded['exp_id'],
        'action_id': self.data_recorded['action_id'],
        'object_id': self.data_recorded['object_id'],
        'action': self.data_recorded['action'],
        'tote_num': self.data_recorded['tote_num'],
        'frame_rate_ratio': self.data_recorded['frame_rate_ratio'],
        'image_size': self.data_recorded['image_size'],
        'path': self.data_recorded['directory'] + '/' + str(self.data_recorded['action_id']),
        'duration': self.stop_time - self.start_time,
        'start_time': self.start_time,
        'end_time': self.stop_time
        }

      #Number of readings by sensor
      for key in self.topic_dict:
          info[key + '_count'] = len(self.data_recorded[key])

      #Grasp successfull?
      try:
          pass
          message = self.data_recorded['grasp_status'][0]
          message = message[0]
          msg_dict = convert_ros_message_to_dictionary(message)
          #print '@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@'
          success = msg_dict['position'][0]
          #print success
          if success > 0:
              print 'Grasp succesfull'
          else:
              print 'Grasp not succesfull'
          info['success'] = success
      except ValueError:
          print ValueError

      return info

  def __update_experiment_info(self):
      info_dict = self.__get_grasp_summary()
      self.experiment_info.append(info_dict)
      #Save
      df = pd.DataFrame(self.experiment_info)
      df.to_csv(path_or_buf=self.directory+'/experiment_summary.csv')
      return


  def start_recording(self, action, action_id, tote_num=1, frame_rate_ratio=10, image_size=-1):
    print '################## RECORDING_NOW ############################'

    action_id = str(action) + '_' + str(action_id)
    self.__reset_vars(action=action, action_id=action_id, tote_num=tote_num, frame_rate_ratio=frame_rate_ratio, image_size=image_size)

    self.start_time = time.time()
    self.subscribers = {}
    for key in self.topic_dict: #We subscribe to every topic in the topic_dict
        topic = self.topic_dict[key]['topic']
        msg_format = self.topic_dict[key]['msg_format']
        self.subscribers[key] = rospy.Subscriber(topic, msg_format, self.__callback, key)

  def stop_recording(self, save_action=True):
    print '################## STOPPING_RECORDING ########################'
    self.stop_time = time.time()

    #We unregister from the topics
    for key in self.subscribers:
        self.subscribers[key].unregister()

    #We delete the support variables for the framerate
    del self.data_recorded['rgb_count']
    del self.data_recorded['depth_count']

    if save_action:
        thread.start_new_thread(self.__save_action, ())
        thread.start_new_thread(self.__update_experiment_info, ())

    return

  def set_object_id(self, object_id):
      self.data_recorded['object_id'] = object_id

  def kill_recorder(self):
      print '[RECORDER]: Saving recording session...'
      df = pd.DataFrame(self.experiment_info)
      df.to_csv(path_or_buf=self.directory+'/experiment_summary.csv')
      print '[RECORDER]: Saving session DONE'

