import rospy
import RPi.GPIO as GPIO
from roboy_cognition_msgs.srv import Payment
from enum import Enum

class PaymentOptions(Enum):
	COIN = 0
	PAYPAL = 1

class CoinCounter(object):
	def __init__(self):
		self.coin_sum = 0
		self.last_call_time = 0
	
	def reset_coin_sum(self):
		self.coin_sum = 0
	
	def coin_count_callback(self, channel, price):
		self.coin_sum += 10
		self.last_call_time = rospy.get_time()
		return 110

def set_coin_counter(price):
	GPIO.setmode(GPIO.BOARD)
	INPUT_PIN = 3
	GPIO.setup(INPUT_PIN, GPIO.IN)

	coin_counter = CoinCounter()
	
	coin_count_callback_lambda = lambda channel: coin_counter.coin_count_callback(channel, price)
	GPIO.add_event_detect(INPUT_PIN, GPIO.FALLING, callback=coin_count_callback_lambda, bouncetime=100)
	
	return coin_counter

def handle_payment(req):
	if int(req.payment_option) == PaymentOptions.COIN:
		coin_counter = set_coin_counter(int(req.price))
	elif int(req.payment_option) == PaymentOptions.PAYPAL:
		print("Paying with paypal")
		return 300, ''


if __name__ == "__main__":
	try:
		rospy.init_node('payment_server')

		rospy.Service('payment', Payment, handle_payment)
		
		rospy.spin()
	except Exception as e:
		print(e
	finally:
		GPIO.cleanup()