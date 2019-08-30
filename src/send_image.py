import requests
import qrcode
import base64
import io

#files = {'image': open('C:/Users/Bilal/Desktop/Luigi_SSBU.png', 'rb')}
#files = {'image': open('C:/Users/Bilal/Desktop/blank.png', 'rb')}
data =  {}
data['flavors'] = ['chocolate', 'vanilla']
data['scoops'] = [1, 1]
data['price'] = 2.5
data['payment_option'] = 1
data['default'] = False

qrcode_text = 'https://www.paypal.me/bilalvural35/umur'
img = qrcode.make(qrcode_text)

buff = io.BytesIO()
img.save(buff, 'png')

encoded = base64.b64encode(buff.getvalue()).decode("ascii")
data['encoded'] = encoded

requests.post('http://localhost:1880/image', data=data)