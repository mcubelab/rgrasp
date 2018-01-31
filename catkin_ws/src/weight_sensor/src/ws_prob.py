#!/usr/bin/env python

'''
Interface to use the weight sensor. To basic functions are to calibrate the
sensors (calibrates all of them) and then to read from a bin
'''
import rospy
from ast import literal_eval
from std_msgs.msg import Float64
import numpy as np
import thread
from std_msgs.msg import Int32

class WeightSensor(object):
    '''Calibrate and Read Weight Sensor'''
    def __init__(self):
        # Set Constants
        self.frame_rate = 16
        self.sensor_count = 6
        self.num_samples = 3 # Filter Length
        self.noManThresh = 1.0 # Threshold for 'no weight'
        sensor_acc = 0.03 # Accuracy (percent full scale range)
        sensor_fsr = 10 # Full Scale Range (kg)
        self.sensor_res = sensor_fsr*sensor_acc*(1000.0/100.0)
        # Tuned by hand depending on accuracy of sensor in system.
        # Increase if confident
        self.lambda_scale = 0.6
        self.zero_calib = [0.0]*self.sensor_count

        # Start subscriber
        self.weight_buffer = [[0 for _ in xrange(self.num_samples)] for _ in xrange(self.sensor_count)]
        self.listener()

        # We start the fall detection publisher to record time during shaking
        self.pub = rospy.Publisher('/ws_drop_detect', Int32, queue_size=1)
        self.drop_detected = True
        self.drop_detection_bin = -1

    def calibrateWeights(self, withSensor=True):
        '''Compute baseline for tote weights and zero out readings,
        With the option to call this in simulation)
        @param sim (bool) Flag on real or not'''

        print '[WS] Calibrating . .',
        if withSensor:
            rospy.sleep(self.num_samples/self.frame_rate)
            self.zero_calib = np.average(self.weight_buffer, 1).tolist()
        else:
            self.zero_calib = [0.0]*self.sensor_count
        print '. . complete!'

    def readWeightSensor(self, item_list, withSensor=True, binNum=0,
                         givenWeights=0):
        '''Operate weight sensor
        @param item_list (list) List of items in bin
        @param withSensor(bool) Real or simulated sensor
        @param binNum (int) Number of bin to measure
        @param givenWeights (int) Simulated weight
        @return ws_data (dict) Probabilities and weights'''
        if binNum < 0 or binNum > 8:
            raise ValueError('The bin number must be between 0 and 8')

        item_list.append('no_item')
        realBinNum = self.convertBinNum(binNum)

        # After compute the weight deficits found from the sensors
        weight = self.subtracted_weight(realBinNum,
                            real=withSensor, sim_weight=givenWeights)
        # Compute the liklihood vector of objects
        probs = self.likelihood_compute(weight, item_list, binNum)
        ws_data = {'probs':probs, 'weights':weight}
        return ws_data

    def listener(self):
        for i in xrange(self.sensor_count):
            rospy.Subscriber("/ws_stream{}".format(i), Float64, self.callback, i)

    def callback(self, data, stream_number):
        (self.weight_buffer[stream_number]).pop(0)
        (self.weight_buffer[stream_number]).append(data.data)

        #Detecting object drop
        if self.drop_detected == False:
            self.__identify_impact()


    def subtracted_weight(self, binNum,
                          real=True,
                          sim_weight=0):
        '''Find the resulting weight after removing an object
        @param binNum (int) Number of bin to query weight
        @return subtracted_weights Weight Difference'''

        if real:
            rospy.sleep(self.num_samples/self.frame_rate)
            weight = np.average(self.weight_buffer[binNum])
        else:
            weight = sim_weight

        subtracted_weights = self.zero_calib[int(binNum)] - weight

        # Check to make sure an item was retrieved
        # else the numbers will be bad
        if subtracted_weights < self.noManThresh:
            print '[WS] No change detected'

        return subtracted_weights

    def prob_gen(self, measured, obj_weight):
        '''compute liklihood for each measured deficient and
        object weight pair'''
        return np.exp(-np.abs(np.floor((measured-obj_weight))/self.sensor_res)*self.lambda_scale)

    def likelihood_compute(self, measured_weight, item_list, binNum):
        '''Compute the likelihood for all objects
        @param measured_weight Measured weight of difference
        @param item_list List of items in the bin
        @param binNum User inputted bin number (used for logging)
        @return prob_vec_norm Probability of each item'''

        obj_weights = []
        for x in xrange(len(item_list)):
            item_name = (item_list[x]).lower()
            if item_name == 'no_item':
                w = 0.0
            else:
                w = rospy.get_param('/obj/{}/weight'.format(item_name))
            obj_weights.append(float(w)*1000)

        prob_vec = [self.prob_gen(measured_weight, x) for x in obj_weights]
        normalization_factor = sum(prob_vec)+1e-6
        prob_vec_norm = np.divide(prob_vec, normalization_factor)

        print '[WS] Measured weight is {:6.2f} g'.format(measured_weight),
        print 'in bin {}\n[WS]'.format(binNum),
        wp = zip(item_list, obj_weights, prob_vec_norm)
        wp.sort(key=lambda x: x[2], reverse=True)
        for i in xrange(len(wp)):
            if wp[i][2] > 0.3 or i < 3:
                print '{} - {:5.4}g - {:.1%} /'.format(wp[i][0],
                         wp[i][1], wp[i][2]),
        print
        return prob_vec_norm

    def convertBinNum(self, original_binNum):
        '''There is not a one to one matching of bins to ws_stream
        due to the current APC layout
        @param original_binNum Bin number given by the user
        @return (no name) Converted bin number to match ws_stream'''
        if original_binNum < 4:
            return original_binNum
        elif original_binNum == 4 or original_binNum == 5:
            return 0
        elif original_binNum == 6 or original_binNum == 7:
            return 4
        elif original_binNum == 8:
            return 5
        else:
            raise ValueError('Impossible bin number input')

    def __identify_impact(self):
        binNum = self.drop_detection_bin
        if binNum == -1 or len(self.weight_buffer[binNum]) < 3:
            return

        # 1. We get the mean of the first n-2 readings
        first_mean = 0
        n = len(self.weight_buffer[binNum])-2
        for index in range(len(self.weight_buffer[binNum])-2):
            first_mean += self.weight_buffer[binNum][index]/(len(self.weight_buffer[binNum])-2)

        last_mean = 0
        for index in range(2):
            index = len(self.weight_buffer[binNum])-1-index
            last_mean += self.weight_buffer[binNum][index]/2

        if ((last_mean - first_mean)/2) > self.threshold:
            self.drop_detected = True
            print "[WS DROP LISTENER]: Drop detected!"
            self.pub.publish(1)


    def start_listening_for_drop(self, bin_num, threshold=50):
        print "[WS DROP LISTENER]: START listening for drop called"
        self.drop_detected = False
        self.drop_detection_bin = self.convertBinNum(bin_num)
        self.threshold = threshold

    def stop_listening_for_drop(self):
        print "[WS DROP LISTENER]: STOP listening for drop called"
        if self.drop_detected == False:
            self.drop_detected = True
            self.pub.publish(0)
            print "[WS DROP LISTENER]: Drop NOT detected!"
        self.drop_detected = True


if __name__ == "__main__":
    rospy.init_node('ws_prob')
