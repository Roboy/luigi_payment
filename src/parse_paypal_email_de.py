#!/usr/bin/env python

import imaplib
import mailparser
import re

#Sie haben Geld erhalten
email_user = 'roboyicecream@outlook.com'


import getpass
email_pass = getpass.getpass('Password:')


try:
    mail = imaplib.IMAP4_SSL('imap-mail.outlook.com', 993)
    mail.login(email_user, email_pass)
    mail.select()
    
    PAYPAL_LANGUAGE = 'DE'

    if PAYPAL_LANGUAGE == 'DE':
        typ, data = mail.search(None, '(FROM "service@paypal.de" SUBJECT "Sie haben Geld erhalten")')
    elif PAYPAL_LANGUAGE == 'EN':
        typ, data = mail.search(None, '(FROM "service@paypal.de" SUBJECT "You\'ve got money")')
    
    #typ, data = mail.search(None, '(FROM "service@paypal.de" SUBJECT "Sie haben Geld erhalten")')
    print(len(data[0].split()))
    if typ == 'OK':
        for num in data[0].split():
            typ2, data2 = mail.fetch(num, '(RFC822)')
            raw_email = data2[0][1]
            str_email = mailparser.parse_from_bytes(raw_email)
            
            body_str = str_email.text_html[0]
            # Parsing 'Rafael Hostettler sent you 0,02 Euro.'.
            if PAYPAL_LANGUAGE == 'DE':
                name_end_pos = body_str.find('hat Ihnen') - 1
            elif PAYPAL_LANGUAGE == 'EN':
                name_end_pos = body_str.find('sent you') - 1
            name_start_pos = body_str.find('>', name_end_pos-50, name_end_pos) + 1
            print(body_str[name_start_pos:name_end_pos])
            money_area_end = body_str.find('<', name_end_pos)
            money_area = body_str[name_end_pos:money_area_end]
            #print(money_area)
            if 'EUR' in money_area:
                money = re.findall(r'\d[,\d]*', money_area)[0].replace(',','.')
                print(money)
        with open('email.txt', 'w') as fp:
            fp.write(body_str)
                

except Exception as e:
    print(e)

finally:
    mail.close()
    mail.logout()