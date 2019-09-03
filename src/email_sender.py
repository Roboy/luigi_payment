#!/usr/bin/env python

from email.MIMEMultipart import MIMEMultipart
from email.MIMEText import MIMEText

with open('mock_credentials.txt', 'r') as f:
	fromaddr = f.readline().strip()
	password = f.readline().strip()

#fromaddr = "luigimockup@outlook.com"

toaddr = "roboyicecream@outlook.com"

msg = MIMEMultipart()

msg['From'] = fromaddr
msg['To'] = toaddr
#msg['Subject'] = "You\'ve got money"
msg['Subject'] = "Sie haben Geld erhalten"

with open('email.txt', 'r') as fp:
	body = fp.read()
msg.attach(MIMEText(body, 'html'))

import smtplib
server = smtplib.SMTP('smtp.live.com', 25)
server.ehlo()
server.starttls()
server.ehlo()

import getpass
#password = getpass.getpass()


server.login(fromaddr, password)
text = msg.as_string()
server.sendmail(fromaddr, toaddr, text)