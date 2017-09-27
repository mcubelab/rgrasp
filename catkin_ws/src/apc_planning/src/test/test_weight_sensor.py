import sys
sys.path.append('/home/mcube/arc/catkin_ws/src/weight_sensor/src')
import ws_prob
import rospy

rospy.init_node('weight_sensor_rosout_listener', anonymous=True)
W = ws_prob.WeightSensor()

W.calibrateWeights(withSensor=True)
raw_input('xx')
weight_info = W.readWeightSensor([], withSensor=True, binNum=0)

print 'weight: ', weight_info['weights']
