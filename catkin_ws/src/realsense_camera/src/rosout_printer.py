#!/usr/bin/env python
import rospy
from rosgraph_msgs.msg import Log

def callback(data):
    if data.level >= Log.INFO and data.name == '/arc_2/stream2':
        print '[ INFO] [' +str(data.header.stamp.secs) + '.' + ('%09d' % (data.header.stamp.nsecs)) +']                                                                 | arc2: ', data.msg
    
def listener():
    rospy.init_node('realsense_rosout_listener', anonymous=True)
    rospy.Subscriber("/rosout_agg", Log, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
