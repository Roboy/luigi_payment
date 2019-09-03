#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from roboy_cognition_msgs.srv import Payment
from enum import IntEnum
import qrcode
import base64
import imaplib
import mailparser
import re
import sys
import requests
import io


# Every signal takes:
# 20 ms in fast mode
# 50 ms in medium mode
# 100 ms in slow mode
MAX_COIN_WAIT_TIME = 30 # in seconds
MAX_PAYPAL_WAIT_TIME = 60 # in secons
INPUT_PIN = 3 # Raspberry Pi GPIO pin to read coin counter output.
EXTRA_WAITING_TIME = 10 # in seconds
PRICE_CHECK_INTERVAL = 1 # in seconds
PAYPAL_ME_URL = 'https://www.paypal.me/bilalvural35/'


class PaymentOptions(IntEnum):
	COIN = 0
	PAYPAL = 1


class CoinCounter(object):
	def __init__(self):
		self.coin_sum = 0
		self.last_call_time = 0
	
	def coin_count_callback(self, channel):
		self.coin_sum = self.coin_sum + 10 # Every signal is 10 cents.
		self.last_call_time = rospy.get_time()

class PaypalAccount(object):
	def __init__(self):
		self.mail = None
		self.username = None
		self.password = None
	
	def init_mail(self):
		# 1st line username
		# 2nd line password
		with open('credentials.txt', 'r') as f:
			self.username = f.readline().strip()
			self.password = f.readline().strip()
		
		# Connecting to e-mail provider.
		self.mail = imaplib.IMAP4_SSL('imap-mail.outlook.com', 993)
		
		# Logging in to e-mail account.
		self.mail.login(self.username, self.password)
		
		# Selecting all e-mails.
		self.mail.select()

		rospy.logdebug('Successfully initialized e-mail!')
	
	def get_num_mail(self):
#		ret_val, mail_ids = self.mail.search(None, '(FROM "service@paypal.de" SUBJECT "You\'ve got money")')
#		ret_val, mail_ids = self.mail.search(None, '(FROM "bilal_v@hotmail.com" SUBJECT "You\'ve got money")')
		ret_val, mail_ids = self.mail.search(None, '(FROM "luigimockup@outlook.com" SUBJECT "You\'ve got money")')
		if ret_val == 'OK':
			return len(mail_ids[0].split())
		else:
			return None
	
	def get_last_payment(self):
#		ret_val_search, mail_ids = self.mail.search(None, '(FROM "service@paypal.de" SUBJECT "You\'ve got money")')
#		ret_val_search, mail_ids = self.mail.search(None, '(FROM "bilal_v@hotmail.com" SUBJECT "You\'ve got money")')
		ret_val_search, mail_ids = self.mail.search(None, '(FROM "luigimockup@outlook.com" SUBJECT "You\'ve got money")')
		
		if ret_val_search == 'OK':
			# Getting last e-mail id.
			last_mail_id = mail_ids[0].split()[-1]
			# Getting data of the last e-mail.
			ret_val_fetch, mail_data = self.mail.fetch(last_mail_id, '(RFC822)')

			if ret_val_fetch == 'OK':
				# Raw e-mail is bytes.
				raw_email = mail_data[0][1]
				
				# For Python 2: parse string.
				if sys.version_info[0] < 3:
					str_email = mailparser.parse_from_string(raw_email)
				# For Python 3: From bytes to string.
				else:
					str_email = mailparser.parse_from_bytes(raw_email)
				
				# PayPal e-mails does not contain plain text area.
				# str_emil.text_plain returns empty.
				# Thus we are getting HTML form of the e-mail.
				body_str = str_email.text_html[0]
				
				# Parsing 'Rafael Hostettler sent you 0,02 Euro.'.
				name_end_pos = body_str.find('sent you') - 1
				# Assuming sum of characters in name and surname
				# should not be more than 50 characters.
				name_start_pos = body_str.find('>', name_end_pos-50, name_end_pos) + 1
				
				# Full name of the sender.
				sender_name = body_str[name_start_pos:name_end_pos]
				rospy.logdebug(sender_name + ' is the client.')
				
				# Parsing money part.
				money_area_end = body_str.find('<', name_end_pos)
				money_area = body_str[name_end_pos:money_area_end]
				
				if 'EUR' in money_area:
					# Finds both 2 and 0,02
					money = re.findall(r'\d[,\d]*', money_area)[0].replace(',','.')
					# Money from Euro to Cents
					return float(money) * 100, sender_name, ''
				else:
					return 0, sender_name, 'Unknown currency.'
			else:
				return 0, '', 'Internal mail error.'
		else:
			return 0, '', 'Internal mail error.'

def show_ads_on_tablet():
	data = {'default': True}
	requests.post('http://localhost:1880/image', data=data)

def show_order_on_tablet(flavors, scoops, price, payment_option, encoded_img=None):
	data = {'flavors': flavors, 'scoops': scoops, 'price': price, 'payment_option': payment_option, 'default': False}
	
	if payment_option == PaymentOptions.PAYPAL:
		data['encoded'] = encoded_img
		data['timer'] = MAX_PAYPAL_WAIT_TIME
	elif payment_option == PaymentOptions.COIN:
		data['timer'] = MAX_COIN_WAIT_TIME
	
	requests.post('http://localhost:1880/image', data=data)

def handle_payment(req, coin_counter, paypal_acc):
	try:
		if int(req.payment_option) == PaymentOptions.COIN:
			rospy.logdebug('Coin has selected for payment.')
			
			rospy.loginfo('You have ' + str(MAX_COIN_WAIT_TIME) + ' seconds to insert coins!')
			
			starting_time = rospy.get_time()

			# Reset coin_sum for every service call.
			coin_counter.coin_sum = 0

			# Show the order on tablet.
			show_order_on_tablet(req.flavors, req.scoops, int(req.price), int(req.payment_option))

			# Check paid amount every second.
			total_slept_time = 0
			while coin_counter.coin_sum < req.price and total_slept_time < MAX_COIN_WAIT_TIME:
				rospy.sleep(PRICE_CHECK_INTERVAL)
				total_slept_time = total_slept_time + PRICE_CHECK_INTERVAL
			
			# Before returning earlier than maximum wait time, wait for stable coin reader.
			# Otherwise it can return less amount than actually paid.
			if total_slept_time < MAX_COIN_WAIT_TIME:
				while rospy.get_time() - coin_counter.last_call_time < PRICE_CHECK_INTERVAL:
					rospy.sleep(PRICE_CHECK_INTERVAL)

					# Do not wait too much.
					# Otherwise users can bully us.
					# Bullies should lose their money.
					if rospy.get_time() - starting_time > MAX_COIN_WAIT_TIME + EXTRA_WAITING_TIME:
						break
			
			rospy.logdebug('Payment server returned ' + str(MAX_COIN_WAIT_TIME - total_slept_time) + ' seconds earlier.')
			rospy.loginfo('You have paid ' + str(coin_counter.coin_sum) + ' cents.')
			
			# Show advertisement on tablet.
			show_ads_on_tablet()

			return coin_counter.coin_sum, '', ''
		
		elif int(req.payment_option) == PaymentOptions.PAYPAL:
			rospy.logdebug('PayPal has selected for payment.')
			rospy.loginfo('You have ' + str(MAX_PAYPAL_WAIT_TIME) + ' seconds to pay with PayPal!')
			
			# Calculating price as Euros and Cents.
			price_eur = int(req.price) // 100
			price_cent = int(req.price) % 100
			
			qrcode_text = PAYPAL_ME_URL + str(price_eur) + '.' + str(price_cent) + 'EUR'
			rospy.logdebug('Creating QR Code with the following link: ' + qrcode_text)
			
			img = qrcode.make(qrcode_text)
			rospy.logdebug('QR Code has generated!')

			# Save QR Code image to buffer.
			buff = io.BytesIO()
			img.save(buff, 'png')

			# Encode buffered image.
			encoded_img = base64.b64encode(buff.getvalue()).decode("ascii")
			
			rospy.logdebug('QR Code has converted to base64!')

			# Show the order on tablet.
			show_order_on_tablet(req.flavors, req.scoops, int(req.price), int(req.payment_option), encoded_img=encoded_img)
			
			# Number of mails before payment process.
			mail_sum_prev = paypal_acc.get_num_mail()
			
			# Check for payment mails every second.
			total_slept_time = 0
			while paypal_acc.get_num_mail() <= mail_sum_prev and total_slept_time < MAX_PAYPAL_WAIT_TIME:
				rospy.sleep(PRICE_CHECK_INTERVAL)
				total_slept_time = total_slept_time + PRICE_CHECK_INTERVAL
			
			# If new payment mail received, return last payment.
			if paypal_acc.get_num_mail() > mail_sum_prev:
				money, sender_name, msg = paypal_acc.get_last_payment()
				rospy.loginfo('You have paid ' + str(money) + ' cents.')
				
				# Show advertisement on tablet.
				show_ads_on_tablet()
				
				return money, sender_name, msg
			else:
				# Show advertisement on tablet.
				show_ads_on_tablet()
				
				return 0, '', 'No payment.'
		else:
			# Show advertisement on tablet.
			show_ads_on_tablet()

			return 0, '', 'Unknown payment option.'
	
	except Exception as e:
		if int(req.payment_option) == PaymentOptions.COIN:
			# Show advertisement on tablet.
			show_ads_on_tablet()

			return coin_counter.coin_sum, '', str(e)
		else:
			# Show advertisement on tablet.
			show_ads_on_tablet()
			
			return 0, '', str(e)


if __name__ == "__main__":
	try:
		# Settings for Raspberry Pi
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(INPUT_PIN, GPIO.IN)
		
		# Initializing payment methods.
		coin_counter = CoinCounter()
		paypal_acc = PaypalAccount()
		paypal_acc.init_mail()
		
		GPIO.add_event_detect(INPUT_PIN, GPIO.FALLING, callback=coin_counter.coin_count_callback, bouncetime=100)

		rospy.init_node('payment_server', anonymous=True, log_level=rospy.DEBUG)
		rospy.logdebug('Payment server has initialized!')
		
		# Using lambda function to let "handle_payment"
		# handle more arguements.
		handle_payment_lambda = lambda req: handle_payment(req, coin_counter, paypal_acc)

		rospy.Service('payment', Payment, handle_payment_lambda)
		rospy.logdebug('Payment server is set!')

		# Set default view for paymant interface.
		show_ads_on_tablet()
		
		rospy.spin()
	
	except Exception as e:
		rospy.logerr(str(e))
	
	finally:
		GPIO.cleanup()
		paypal_acc.mail.close()
		paypal_acc.mail.logout()