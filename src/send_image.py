import requests

#files = {'image': open('C:/Users/Bilal/Desktop/Luigi_SSBU.png', 'rb')}
files = {'image': open('C:/Users/Bilal/Desktop/blank.png', 'rb')}
data =  {}
data['flavors'] = ['chocolate', 'vanilla']
data['scoops'] = [1, 1]
data['price'] = 2.5
data['payment_option'] = 1
data['default'] = False
print(data)
requests.post('http://localhost:1880/image', files = files, data=data)