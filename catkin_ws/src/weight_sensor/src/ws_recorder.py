import rospy, signal, sys
from std_msgs.msg import Float64
from std_msgs.msg import String
import matplotlib.pyplot as plt

ws_lectures = []

def callback(data):
    ###Reads and stores the messages published on the ws_stream
    print data.data
    ws_lectures.append(data.data)


def ws_listener():
    ###Creates a ROS node and subscribes to the ws_stream
    rospy.init_node('ws_listener', anonymous=True)
    
    rospy.Subscriber('ws_stream{}'.format(0), Float64, callback) #Tote 1
    #rospy.Subscriber('ws_stream{}'.format(1), Float64, callback) #Tote 0
    #rospy.Subscriber('ws_stream{}'.format(2), Float64, callback) #Tote 2
    
    rospy.spin() # This keeps python from exiting until this node is stopped


def exit_gracefully(signum, frame):
    ###Plots the values and stores them
    signal.signal(signal.SIGINT, original_sigint)
    
    #Saving
    values_file = open('ws_values.txt', 'w')
    for item in ws_lectures:
        values_file.write("%s\n" % item)
    print ''
    print 'Weight evolution saved'
    
    #Plotting
    plt.plot(ws_lectures)
    plt.show()
    
    sys.exit(1)

if __name__ == '__main__':
    global ws_lectures
    
    original_sigint = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, exit_gracefully)
    ws_listener()
