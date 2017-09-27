#!/usr/bin/env python
import rospy
from rosgraph_msgs.msg import Log
import datetime

def callback(data):
    if data.level >= Log.INFO and data.name == '/weight_sensor_node':
        print '[ INFO] [' +datetime.datetime.fromtimestamp(data.header.stamp.secs).strftime('%Y%m%d %H:%M:%S') + '.' + ('%02d' % (data.header.stamp.nsecs//10000000)) +']', data.msg
    
def listener():
    rospy.init_node('weight_sensor_rosout_listener', anonymous=True)
    rospy.Subscriber("/rosout_agg", Log, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
