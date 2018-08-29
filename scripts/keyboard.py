import rospy
from std_msgs.msg import Int16
import sys, select, termios, tty

# launch this file to have an emergency button, when press '.' it changes his flag
# from false to true or viceversa. 'q' button is for exit.

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__== "__main__" :
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('key', Int16, queue_size = 1)
    rospy.init_node('keyboard')
    try:
        while(1):
            chiave = getKey()
            if(chiave[0] == 'q'):
                break
            if(chiave[0] == '.'):
                pub.publish(Int16(12))
    except Exception as e:
        print(e)
