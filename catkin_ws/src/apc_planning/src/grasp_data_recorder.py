import rospy, signal, sys, time, os, cv2, pickle, collections
from std_msgs.msg import Int32, Float64, Float32MultiArray, String, Bool
from pr_msgs.msg import gelsight_contactarea
from sensor_msgs.msg import JointState, Image, CompressedImage
from wsg_50_common.msg import Status
from cv_bridge import CvBridge, CvBridgeError
from msg_to_dict import convert_ros_message_to_dictionary
from ssh_helper import ssh
from multiprocessing import Process
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import thread
import gripper, spatula
import datetime

class GraspDataRecorder:
  ##This class records the data gathered through the sensors
  def __init__(self, directory, exp_id=''):
    self.bridge = CvBridge()

    if exp_id == '':
        exp_id = 'EXP_' + datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    self.exp_id = exp_id
    self.directory = directory + '/' + str(exp_id)

    #Directory creation
    if not os.path.exists(self.directory):
      os.makedirs(self.directory)

    self.experiment_info = []
    self.data_recorded = None
    return

  def __reset_vars(self, action, action_id, tote_num, frame_rate_ratio, image_size):
    #Dictionary with all the topics that we will need to subscribe (HARDCODED)
    self.topic_dict = {
                    'ws_0': {'topic':'ws_stream{}'.format(0), 'msg_format':Float64},
                    'ws_1': {'topic':'ws_stream{}'.format(1), 'msg_format':Float64},
                    'ws_2': {'topic':'ws_stream{}'.format(2), 'msg_format':Float64},
                    'gs_image': {'topic':'rpi/gelsight/raw_image', 'msg_format':Image},
#                    'gs_image_compressed': {'topic':'rpi/gelsight/raw_image/compressed', 'msg_format':CompressedImage},
                    'gs_deflection': {'topic':'rpi/gelsight/deflection', 'msg_format':Int32},
                    'gs_contactarea': {'topic':'rpi/gelsight/contactarea', 'msg_format':gelsight_contactarea}, # Reformat DONE
                    'hand_commands': {'topic':'/hand_commands', 'msg_format':JointState}, # Reformat
                    'grasp_status': {'topic':'/grasp_status', 'msg_format':JointState}, # Reformat
                    'joint_states':{'topic':'/joint_states', 'msg_format':JointState}, # Reformat
                    'grasp_all_proposals': {'topic':'/grasp_all_proposals', 'msg_format':Float32MultiArray},
                    'grasp_proposal': {'topic':'/grasp_proposal', 'msg_format':Float32MultiArray},
                    'grasp_noise': {'topic':'/grasp_noise', 'msg_format':Float32MultiArray},
                    'im_input_color_0': {'topic':'/im_input_color_0', 'msg_format':Image},
                    'im_back_color_0': {'topic':'/im_back_color_0', 'msg_format':Image},
                    'im_input_depth_0': {'topic':'/im_input_depth_0', 'msg_format':Image},
                    'im_back_depth_0': {'topic':'/im_back_depth_0', 'msg_format':Image},
                    'im_input_color_1': {'topic':'/im_input_color_1', 'msg_format':Image},
                    'im_back_color_1': {'topic':'/im_back_color_1', 'msg_format':Image},
                    'im_input_depth_1': {'topic':'/im_input_depth_1', 'msg_format':Image},
                    'im_back_depth_1': {'topic':'/im_back_depth_1', 'msg_format':Image},
                    'rgb_bin0': {'topic':'/arc_1/rgb_bin0', 'msg_format':Image},
                    'rgb_bin1': {'topic':'/arc_1/rgb_bin1', 'msg_format':Image},
                    'rgb_bin2': {'topic':'/arc_1/rgb_bin2', 'msg_format':Image},
                    'depth_bin0': {'topic':'/arc_1/depth_bin0', 'msg_format':Image},
                    'depth_bin1': {'topic':'/arc_1/depth_bin1', 'msg_format':Image},
                    'depth_bin2': {'topic':'/arc_1/depth_bin2', 'msg_format':Image},
                    'wsg_driver': {'topic':'/wsg_50_driver/status', 'msg_format':Status}, # Reformat DONE
                    'exp_comments': {'topic':'/exp_comments', 'msg_format':String},
                    'impact_time': {'topic':'/impact_time', 'msg_format':Bool},
                    'objectList': {'topic':'/objectList', 'msg_format':Float32MultiArray},
                    'objectType': {'topic':'/objectType', 'msg_format':String},
                    'liftoff_time': {'topic':'/liftoff_time', 'msg_format':String}
                    }

    #We delete the sensors we do not want to record
    if tote_num == 0:
        del self.topic_dict['ws_1']
        del self.topic_dict['ws_2']
        try:
            del self.topic_dict['rgb_bin1']
            del self.topic_dict['rgb_bin2']
        except:
            pass
        try:
            del self.topic_dict['depth_bin1']
            del self.topic_dict['depth_bin2']
        except:
            pass
    elif tote_num == 1:
        del self.topic_dict['ws_0']
        del self.topic_dict['ws_2']
        try:
            del self.topic_dict['rgb_bin0']
            del self.topic_dict['rgb_bin2']
        except:
            pass
        try:
            del self.topic_dict['depth_bin0']
            del self.topic_dict['depth_bin2']
        except:
            pass

    #We create the dictionary that will host all the info
    self.data_recorded = {'exp_id': self.exp_id, 'action':action, 'action_id':action_id, 'tote_num':tote_num, 'frame_rate_ratio':frame_rate_ratio, 'rgb_count':0, 'depth_count':0, 'image_size':image_size, 'object_id':'', 'directory':self.directory}
    for key in self.topic_dict:
        self.data_recorded[key] = []

  def __callback(self, data, key):
    if key == 'grasp_status' or key =='objectType':
        self.subscribers[key].unregister()

    ##This function is called everytime a msg is published to one of our subscribed topics, it stores the data
    if self.topic_dict[key]['msg_format'] == Image:
        if key == 'rgb_bin0' or key=='rgb_bin1':
            self.data_recorded['rgb_count'] +=1
            if self.data_recorded['rgb_count']%self.data_recorded['frame_rate_ratio'] == 0:
                try:
                    cv2_img = self.bridge.imgmsg_to_cv2(data, 'rgb8') # Convert your ROS Image message to OpenCV2
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
                    cv2_img = self.bridge.imgmsg_to_cv2(data, 'mono16') # Convert your ROS Image message to OpenCV2
                    if self.data_recorded['image_size'] != -1:
                        cv2_img = cv2.resize(cv2_img, self.data_recorded['image_size'])
                except CvBridgeError, e:
                    print(e)
                else:
                    self.data_recorded[key].append((cv2_img, rospy.get_time()))
        else:
            try:
                cv2_img = self.bridge.imgmsg_to_cv2(data, 'rgb8') # Convert your ROS Image message to OpenCV2
                if self.data_recorded['image_size'] != -1:
                    cv2_img = cv2.resize(cv2_img, self.data_recorded['image_size'])
            except CvBridgeError, e:
                print(e)
            else:
                self.data_recorded[key].append((cv2_img, rospy.get_time()))
    elif self.topic_dict[key]['msg_format'] == JointState:
        message = data #Maybe data.data
        data_dict = {}
        data_dict['header'] = str(message.header)
        data_dict['name'] = message.name
        data_dict['position'] = message.position
        data_dict['velocity'] = message.velocity
        data_dict['effort'] = message.effort
        self.data_recorded[key].append((data_dict, rospy.get_time()))
    elif key == 'wsg_driver': # We process the ws_50 msg so that we dont have to import it in mcube learning
        ws50message = data
        data_dict = {}
        data_dict['status'] = ws50message.status
        data_dict['width'] = ws50message.width
        data_dict['speed'] = ws50message.speed
        data_dict['acc'] = ws50message.acc
        data_dict['force'] = ws50message.force
        data_dict['force_finger0'] = ws50message.force_finger0
        data_dict['force_finger1'] = ws50message.force_finger1
        self.data_recorded[key].append((data_dict, rospy.get_time()))
    elif key == 'gs_contactarea':
        self.data_recorded[key].append((data.contact_map, rospy.get_time()))
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

      print self.data_recorded['grasp_status']
      #We save all the readings from the sensors
      for key in self.topic_dict:
          #print key
          values_filename = str(key) + '_values'
          timestamps_filename = str(key) + '_timestamps'
          try:
            values, timestamps = zip(*self.data_recorded[key])
            np.savez_compressed(path + '/' + values_filename, values)
            np.savez_compressed(path + '/' + timestamps_filename, timestamps)
          except Exception as e:
            print e


      #pickle.dump(self.data_recorded, open(path, "wb"))
      print '[RECORDER]: Saving DONE'

      # thread.start_new_thread(self.__send_to_server, ()) #TO SAVE THE FOLDER AT THE SERVER TOO

  def __send_to_server(self):
      local_path = self.data_recorded['directory'] + '/' + str(self.data_recorded['action_id'])
      remote_path = 'media/mcube/SERVER_HD/Dropbox (MIT)/rgrasp_dataset/'

      s = ssh(computer_id='server')

      # If the Experiment directory doesn't exist on the server we create it (for first time saving)
      dir_list = s.get_dir_list(remotepath=remote_path)
      if str(self.data_recorded['exp_id']) not in dir_list:
          s.mkdir(remote_path + str(self.data_recorded['exp_id']))

      s.put_dir(localpath=local_path, remotepath=remote_path+str(self.data_recorded['exp_id']))

  def __get_grasp_summary(self):
      print '[RECORDER]: Building summary'

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
        'end_time': self.stop_time,
        'event_dict': self.__get_event_dict()
        }

      #Number of readings by sensor
      for key in self.topic_dict:
          info[key + '_count'] = len(self.data_recorded[key])

      #Grasp successfull?
      if self.data_recorded['grasp_status'] == []:
          return info

      try:
          message = self.data_recorded['grasp_status'][0][0]
          success = message['position'][0]
          if success > 0:
              print '++++++++++++++++++++++++++++++++++++++++++++++++++GRASP SUCCESFULL'
          else:
              print '++++++++++++++++++++++++++++++++++++++++++++++++++GRASP  NOT  SUCCESFULL'
          info['success'] = success
      except Exception as e:
          print '[RECORDER]: ERROR FINDING SUCCESS:'
          print e

      try:
          val, timestamp = zip(*self.data_recorded['objectType'])
          info['object_id'] = val[0]
      except Exception as e:
          print '[RECORDER]: ERROR WITH OBJECT IDS:'
          print e

      return info

  def __update_experiment_info(self):
      info_dict = self.__get_grasp_summary()
      self.check_sensors(info_dict)
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

  def update_topic(self, key):
    #if key == 'grasp_status':
        #print 'GRASP STATUS RECEIVED!!!!!!!!!!!!!!!!!!!!!!!!!!'
    #This function changes the value of the given topic for the msg and saves it
    #Replacing the .npz file
    #msg = rospy.Subscriber(topic, msg_format, self.__callback, key)
    if self.data_recorded == None:
        return

    try:
        self.data_recorded[key] = [] #We erase the data recorded from this topic
        topic = self.topic_dict[key]['topic']
        msg_format = self.topic_dict[key]['msg_format']
        self.subscribers[key] = rospy.Subscriber(topic, msg_format, self.__callback, key) #We download it again

        time.sleep(1)
    except Exception as e:
        print 1
        print e

    try:
        path = self.data_recorded['directory'] + '/' + str(self.data_recorded['action_id'])
        values_filename = str(key) + '_values'
        timestamps_filename = str(key) + '_timestamps'
        values, timestamps = zip(*self.data_recorded[key])
        np.savez_compressed(path + '/' + values_filename, values)
        np.savez_compressed(path + '/' + timestamps_filename, timestamps)

        #Updating the summary
        #self.data_recorded[key] = msg_list
        self.experiment_info = self.experiment_info[:-1] #We delete the last experiment info entry
        self.__update_experiment_info() #We reenter the last experiment info with the updated data_recorded
        self.kill_recorder() #We save this file
    except Exception as e:
        print 2
        print e


  def kill_recorder(self):
      print '[RECORDER]: Saving recording session...'
      df = pd.DataFrame(self.experiment_info)
      df.to_csv(path_or_buf=self.directory+'/experiment_summary.csv')
      print '[RECORDER]: Saving session DONE'

  def check_sensors(self, info_dict):
      #software stop robot function
      def abort():
          print ('[Recorder] ***************************************************')
          print ('[Recorder] Sensor Issue: The sensor {} is empty. Abort!'.format(term))
          print ('[Recorder] ***************************************************')
          print ('info_dict', info_dict)
          gripper.open()
          spatula.open()
          sys.exit()
      #check if some sensors have count = 0
      for term in info_dict:
          if 'count' in term:
              if ((info_dict[term]==0) and (term not in ['objectList_count', 'wsg_driver_count', 'grasp_status_count', 'objectType_count','im_input_depth_0_count','im_input_depth_1_count','im_back_depth_0_count','im_back_depth_1_count','im_input_color_0_count','im_input_color_1_count','im_back_color_0_count','im_back_color_1_count'])):
                  abort()


  def __get_event_dict(self):
      event_dict = {}
      # Impact time
      for val, timestamp in self.data_recorded['impact_time']:
        event_dict['impact_time'] = timestamp

      for val, timestamp in self.data_recorded['liftoff_time']:
        event_dict['liftoff_time'] = timestamp

      for val, timestamp in self.data_recorded['hand_commands']:
        name = val['name'][0]
        event_dict[name] = timestamp

      return event_dict
