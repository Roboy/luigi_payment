import sys
import rospy
from roboy_cognition_msgs.srv import Payment

def payment_client(price, payment_option):
    try:
        rospy.logdebug('Waiting for payment service...')
        rospy.wait_for_service('payment')
        rospy.logdebug('Payment service is ready!')

        rospy.logdebug('Setting payment service proxy...')
        payment = rospy.ServiceProxy('payment', Payment)
        rospy.logdebug('Payment service proxy is set!')
        
        resp = payment(price, payment_option)
        
        return resp.amount_paid, resp.error_message
    
    except Exception as e:
        rospy.logerr(str(e))

if __name__ == "__main__":
    # Mock values
    price = 600
    payment_option = 0

    rospy.init_node('payment_client', anonymous=True, log_level=rospy.DEBUG) # Optional
    
    while not rospy.is_shutdown():
    	rospy.loginfo('Requesting ' + str(price) + ' cents.')
    	# Example service call.
    	rospy.loginfo('Received payment: ' + str(payment_client(price, payment_option)))