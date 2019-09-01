from email.MIMEMultipart import MIMEMultipart
from email.MIMEText import MIMEText

fromaddr = "bilal_v@hotmail.com"
#fromaddr = "luigimockup@outlook.com"

#toaddr = "vural.bilal@outlook.com"
toaddr = "roboyicecream@outlook.com"

msg = MIMEMultipart()

msg['From'] = fromaddr
msg['To'] = toaddr
msg['Subject'] = "You\'ve got money"

with open('email.txt', 'r') as fp:
	body = fp.read()
msg.attach(MIMEText(body, 'html'))

import smtplib
server = smtplib.SMTP('smtp.live.com', 25)
server.ehlo()
server.starttls()
server.ehlo()

import getpass
password = getpass.getpass()

#password = "Roboy2016"

server.login(fromaddr, password)
text = msg.as_string()
server.sendmail(fromaddr, toaddr, text)