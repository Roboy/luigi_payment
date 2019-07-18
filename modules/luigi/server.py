import rospy
import RPi.GPIO as GPIO
from roboy_cognition_msgs.srv import Payment
from time import time
from enum import Enum
from time import sleep


COIN_WAIT_TIME = 10 # in seconds
INPUT_PIN = 3 # Raspberry Pi GPIO pin to read coin counter output.

class PaymentOptions(Enum):
	COIN = 0
	PAYPAL = 1


class CoinCounter(object):
	def __init__(self):
		self.coin_sum = 0
		self.last_call_time = 0
	
	def coin_count_callback(self, channel):
		self.coin_sum = self.coin_sum + 10 # Every signal is 10 cents.
		self.last_call_time = time()
	
	def handle_payment(self, req):
		try:
			rospy.loginfo('You have ' + str(COIN_WAIT_TIME) + ' seconds to insert coins!')
			
			# Check paid amount every second.
			total_slept_time = 0
			while self.coin_sum < req.price and total_slept_time < COIN_WAIT_TIME:
				sleep(1)
				total_slept_time = total_slept_time + 1
			
			sleep(1) # Wait 1 more second for coin reader stabilization, otherwise it can return less amount 
			return self.coin_sum, ''
		
		except Exception as e:
			return self.coin_sum, str(e)

if __name__ == "__main__":
	try:
		# Settings for Raspberry Pi
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(INPUT_PIN, GPIO.IN)

		coin_counter = CoinCounter()
		
		GPIO.add_event_detect(INPUT_PIN, GPIO.FALLING, callback=coin_counter.coin_count_callback, bouncetime=100)
		
		rospy.init_node('payment_server', anonymous=True, log_level=rospy.DEBUG)
		rospy.logdebug('Payment server has initialized!')
		
		rospy.Service('payment', Payment, coin_counter.handle_payment)
		rospy.logdebug('Payment server is set!')
		
		rospy.spin()
	
	except Exception as e:
		rospy.logerr(str(e))
	
	finally:
		GPIO.cleanup()