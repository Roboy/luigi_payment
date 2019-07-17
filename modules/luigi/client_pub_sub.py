import rospy
from std_msgs.msg import UInt16
from time import sleep


class Client(object):
    def __init__(self):
        self.price = 0
        self.paid_amount = 0

        rospy.init_node('payment_client', anonymous=True, log_level=rospy.DEBUG)
        rospy.logdebug('Payment client has initialized!')

        rospy.Subscriber('paid_amount_publisher', UInt16, self.ros_callback)
        rospy.logdebug('Subscribed to paid_amount_publisher!')

        self.price_publisher = rospy.Publisher('price_publisher', UInt16, queue_size=10)
        rospy.logdebug('price_publisher is set!')

        self.rate = rospy.Rate(60)
    def ros_callback(self, data):
        paid_amount = data.data
        rospy.loginfo(str(paid_amount) + ' cents has been paid!')
    
if __name__ == '__main__':
    try:
        client = Client()
        sleep(5)
        while not rospy.is_shutdown():
            client.price_publisher.publish(400)
            rospy.loginfo('Published price!')
            
            sleep(5)
            
            #client.rate.sleep()
    except Exception as e:
        rospy.logerr(str(e))