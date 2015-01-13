from __future__ import print_function
from datetime import datetime
from time import time
from os import remove

__author__ = 'niclas'
# All debugging and logging messages happen through this script

#logging levels
levels = {0:'Debug', 1: 'Warning', 2: 'Error'}
logfile = '../../config/dronarch.log'
def debug(level,*message):
    """
    All Debug and Error messages should be sent to this function.
    This allows to modify and redirect the messages.
    :param message:
    :param level:
    :return:
    """
    msg = ''.join([str(arg) for arg in message ])
    if level in levels.keys():
        output = levels[level],': ', msg
        print(levels[level],': ', msg)
        write_to_file(logfile=logfile, message=' '.join(output))
    else:
        print('Invalid debug level {} for message "{}". Has to be one of {}'.format(level,msg,levels))

def write_to_file(logfile, message):
    with open(logfile, 'a+') as file:
        timestamp = datetime.fromtimestamp(time()).strftime('%d.%m.%Y %H:%M:%S')
        file.write(timestamp+': '+message+'\n')

def clear_log_file():
    remove(logfile)

if __name__=='__main__':
    debug(1, 'some ', ' more ', 23)
    debug(999, 'A message')
