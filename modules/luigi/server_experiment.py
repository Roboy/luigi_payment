import rospy
import RPi.GPIO as GPIO
from roboy_cognition_msgs.srv import Payment
from enum import Enum
import qrcode
import base64

# Every signal takes:
# 20 ms in fast mode
# 50 ms in medium mode
# 100 ms in slow mode
MAX_COIN_WAIT_TIME = 30 # in seconds
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
		self.last_call_time = rospy.get_time()

def handle_payment(req, coin_counter):
	try:
		if int(req.payment_option) == PaymentOptions.COIN:
			rospy.loginfo('You have ' + str(MAX_COIN_WAIT_TIME) + ' seconds to insert coins!')
			
			starting_time = rospy.get_time()

			# Reset coin_sum for every service call.
			coin_counter.coin_sum = 0

			# Check paid amount every second.
			total_slept_time = 0
			while coin_counter.coin_sum < req.price and total_slept_time < MAX_COIN_WAIT_TIME:
				rospy.sleep(1)
				total_slept_time = total_slept_time + 1
			
			# Before returning earlier than maximum wait time, wait for stable coin reader.
			# Otherwise it can return less amount than actually paid.
			if total_slept_time < MAX_COIN_WAIT_TIME:
				while rospy.get_time() - coin_counter.last_call_time < 1:
					rospy.sleep(1)

					# Do not wait too much.
					# Otherwise users can bully us.
					# Bullies should lose their money.
					if rospy.get_time() - starting_time > MAX_COIN_WAIT_TIME + 10:
						break
			
			rospy.logdebug('Payment server returned ' + str(MAX_COIN_WAIT_TIME - total_slept_time) + ' seconds earlier.')
			rospy.loginfo('You have paid ' + str(coin_counter.coin_sum) + ' cents.')
			
			return coin_counter.coin_sum, ''
		elif int(req.payment_option) == PaymentOptions.PAYPAL:
			img = qrcode.make('https://www.paypal.me/bilalvural35/' + str(req.price))
			rospy.logdebug('QRCode has generated!')
			
			img_str = base64.b64encode(img.tobytes())
			rospy.logdebug('QRCode has converted to base64!')
			return 0, ''
	
	except Exception as e:
		return coin_counter.coin_sum, str(e)

if __name__ == "__main__":
	try:
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(INPUT_PIN, GPIO.IN)

		coin_counter = CoinCounter()
		
		GPIO.add_event_detect(INPUT_PIN, GPIO.FALLING, callback=coin_counter.coin_count_callback, bouncetime=100)

		rospy.init_node('payment_server', anonymous=True, log_level=rospy.DEBUG)
		rospy.logdebug('Payment server has initialized!')
		
		handle_payment_lambda = lambda req: handle_payment(req, coin_counter)

		rospy.Service('payment', Payment, handle_payment_lambda)
		rospy.logdebug('Payment server is set!')
		
		rospy.spin()
	
	except Exception as e:
		rospy.logerr(str(e))
	
	finally:
		GPIO.cleanup()