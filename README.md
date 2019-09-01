# Payment System of Luigi
Repository includes payment server, a mock client, Node-RED dashboard to show interface and sample PayPal email sender script.

## Payment Server
`src/server.py` handles two payment methods which are coins and PayPal.
In order to run it, first start `roscore` then start `server.py` via `python server.py`
* The scripts expect `credentials.txt` file on the same folder which is `src`.
	* 1st line should be e-mail address.
	* 2nd line should be your e-mail password.
* You can change the following lines in `src/server.py` to fit your needs.
```bash
MAX_COIN_WAIT_TIME = 30 # in seconds
MAX_PAYPAL_WAIT_TIME = 60 # in secons
INPUT_PIN = 3 # Raspberry Pi GPIO pin to read coin counter output.
EXTRA_WAITING_TIME = 10 # in seconds
PRICE_CHECK_INTERVAL = 1 # in seconds
PAYPAL_ME_URL = 'https://www.paypal.me/bilalvural35/'
```
> PayPal URL should be paypal.me URL.
## Mock Client
`src/client.py` is going to mock Luigi's payment service call indefinitely with chaning payment options.

## Node-RED Dashboard
You need to start Node-RED via `node-red` and get device's IP to show it on other devices such as tablet.
> Don't forget to import `flows.json` which is under `/node-red-payment-ui` and deploy it after browsing `127.0.0.1:1880`.
> If there are other flows, consider deleting them.
> You can access the dashboard from any device on the same network by browing `SERVER_IP:1880/ui`.

## Sample PayPal E-mail Sender
This script reads scraped real PayPal `email.txt` file and sends it.
* The script expect `mock_credentials.txt` on the same folder which is `src`.
	* 1st line should be e-mail address of the sender.
	* 2nd line should be your e-mail password.
> You can change `toaddr =  "roboyicecream@outlook.com"` to fit your needs.