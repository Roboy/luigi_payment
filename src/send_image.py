import requests
import qrcode
import base64
import io

data =  {}
data['flavors'] = ['chocolate', 'vanilla', 'strawberry']
#data['flavors'] = ['chocolate']

data['scoops'] = [1, 1, 2]
#data['scoops'] = [1]

data['price'] = 250
data['payment_option'] = 1
data['timer'] = 42
data['default'] = False

data['paid'] = 0

qrcode_text = 'https://www.paypal.me/roboyicecream/3.5EUR'
img = qrcode.make(qrcode_text)

buff = io.BytesIO()
img.save(buff, 'png')

encoded = base64.b64encode(buff.getvalue()).decode("ascii")
data['encoded'] = encoded

requests.post('http://localhost:1880/image', data=data)