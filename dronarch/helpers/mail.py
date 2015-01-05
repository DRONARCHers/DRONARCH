__author__ = 'niclas'

import smtplib

"""
TODO: Email adresses and password have to be added
It sends an email as notification.
"""

# your email address
sender = ''
# usually the sender email, but some smtp server have other login names
login = ''
#your/senders smtp password
password = ''
# receivers email address
receiver = ''
# your smtp server
smtp_server=''
#smtp server port. usually 587
port=587

def send_mail(to_adr, subject, msg_content):
    """
    Send an email to specified address using my login.
    :param to_adr:
    :param subject:
    :param msg_content:
    :return:
    """

    server = smtplib.SMTP(smtp_server,port)

    header = 'To:' + to_adr + '\n' + 'From: ' + sender + '\n' + 'Subject: '+subject+'\n'
    msg = header + '\n'+msg_content+' \n\n'
    try:
        server.starttls()
        server.login(login, password)
        server.sendmail(from_addr=sender, to_addrs=to_adr, msg=msg)
        print 'Message "{}" sent'.format(subject)
    except smtplib.SMTPAuthenticationError:
        server.close()
        print 'Could connect to server. Authentication problem'

    server.close()

def send_dronarch_mail(subject, msg_content):
    """
    Sends a dronarch notification. This function is here to hide email addresses and passwords from git,
    because this file should be personalized and then not be published
    :param subject:
    :param msg_content:
    :return:
    """
    send_mail(to_adr=receiver, subject=subject, msg_content=msg_content)

if __name__=='__main__':
    send_mail( to_adr='test_with_your_address',
              subject='Test python script',
              msg_content='This is a test message to check whether this script is working or not.')