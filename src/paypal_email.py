import imaplib
import mailparser
import email
import re

def get_body(b):
    if b.is_multipart():
        for part in b.walk():
            ctype = part.get_content_type()
            cdispo = str(part.get('Content-Disposition'))

            # skip any text/plain (txt) attachments
            if ctype == 'text/plain' and 'attachment' not in cdispo:
                body = part.get_payload(decode=True)  # decode
                break
    # not multipart - i.e. plain text, no attachments, keeping fingers crossed
    else:
        body = b.get_payload(decode=True)
    return body


email_user = 'vural.bilal@outlook.com'
import getpass
email_pass = getpass.getpass('Password:')

try:
    mail = imaplib.IMAP4_SSL('imap-mail.outlook.com', 993)
    mail.login(email_user, email_pass)
    mail.select()

    typ, data = mail.search(None, '(FROM "service@paypal.de" SUBJECT "You\'ve got money")')
    if typ == 'OK':
        for num in data[0].split():
            typ2, data2 = mail.fetch(num, '(RFC822)')
            raw_email = data2[0][1]
            str_email = mailparser.parse_from_bytes(raw_email)
            
            body_str = str_email.text_html[0]
            name_end_pos = body_str.find('sent you') - 1
            money_area_end = body_str.find('<', name_end_pos)
            money_area = body_str[name_end_pos:money_area_end]
            #print(money_area)
            if 'EUR' in money_area:
                money = re.findall(r'\d[,\d]*', money_area)[0].replace(',','.')
                print(money)
                

except Exception as e:
    print(e)

finally:
    mail.close()
    mail.logout()