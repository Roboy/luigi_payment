import imaplib
import mailparser
#import email

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


email_user = 'vural.bilal@outlook.com'#input('Email: ')
import getpass
email_pass = getpass.getpass('Password:')

try:
    mail = imaplib.IMAP4_SSL('imap-mail.outlook.com', 993)
    mail.login(email_user, email_pass)
    mail.select()

    typ, data = mail.search(None, '(FROM "service@paypal.de" SUBJECT "You\'ve got money")')
    if typ == 'OK':
        for num in data[0].split():
            #typ2, data2 = mail.fetch(num, "(UID BODY[TEXT])")
            typ2, data2 = mail.fetch(num, '(RFC822)')
            raw_email = data2[0][1]
            
            new_mail = mailparser.parse_from_bytes(raw_email)
            if len(new_mail.text_plain) != 0:
                print(new_mail.text_plain)
            else:
                print('Text HTML')
                #print(len(new_mail.text_html))
                '''
                body_str = ''.join(new_mail.text_html)
                if 'sent you' in body_str:
                    print(body_str.find('sent you'))
                    print(body_str)
                    break
                '''
            #break
except Exception as e:
    print(e)
finally:
    mail.close()
    mail.logout()