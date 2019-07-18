import rospy
from std_msgs.msg import UInt16
from time import sleep
from time import time
import RPi.GPIO as GPIO


# Every signal takes:
# 20 ms in fast mode
# 50 ms in medium mode
# 100 ms in slow mode
COIN_WAIT_TIME = 30 # in seconds
INPUT_PIN = 3


class Server(object):
    def __init__(self):
        self.coin_sum = 0
        self.last_call_time = 0
        self.ros_call_time = 0
    
    def set(self):
        rospy.init_node('payment_server', anonymous=True, log_level=rospy.DEBUG)
        rospy.logdebug('Payment server has initialized!')

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(INPUT_PIN, GPIO.IN)
        rospy.logdebug('GPIO' + str(INPUT_PIN) + ' is set!')

        GPIO.add_event_detect(INPUT_PIN, GPIO.FALLING, callback=self.coin_count_callback, bouncetime=100)
        rospy.logdebug('GPIO callback is set!')
        
        rospy.Subscriber('price_publisher', UInt16, self.ros_callback)
        rospy.logdebug('Subscribed to price_publisher!')

        self.paid_amount_publisher = rospy.Publisher('paid_amount_publisher', UInt16, queue_size=10)
        rospy.logdebug('paid_amount_publisher is set!')
        
    def coin_count_callback(self, channel):
        self.coin_sum += 10
        self.last_call_time = time()
        #print(self.coin_sum)
    
    def ros_callback(self, data):
        # Blocking other call until current one is finished.
        if time() - self.ros_call_time < COIN_WAIT_TIME:
            rospy.logdebug('Still waiting ' + str(time() - self.ros_call_time) + ' seconds to pass!')
        else:
	        self.coin_sum = 0
	        self.ros_call_time = time()
	        
	        price = data.data
	        
            # Calculating values for prettier info message.
	        eur = price // 100
	        cent = price % 100
	        
	        price_notification_str = 'You must pay '
	        if eur != 0:
	        	price_notification_str = price_notification_str + str(eur) + ' EUR '
	        if cent != 0:
	        	price_notification_str = price_notification_str + str(cent) + ' Cents'
	        
            rospy.loginfo(price_notification_str)
	        rospy.loginfo('You have ' + str(COIN_WAIT_TIME) + ' seconds to insert your coins!')
	        
            # Do not send anything until interval is done.
	        sleep(COIN_WAIT_TIME)
	        
	        self.paid_amount_publisher.publish(self.coin_sum)
	        rospy.logdebug(str(self.coin_sum) + ' cents has published!')
    
if __name__ == '__main__':
    try:
        server = Server()
        server.set()

        rospy.spin()
    
    except Exception as e:
        rospy.logerr(str(e))
    
    finally:
        GPIO.cleanup()