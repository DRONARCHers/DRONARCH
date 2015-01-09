from __future__ import print_function
__author__ = 'niclas'
# All debugging and logging messages happen through this script

#logging levels
levels = {0:'Debug', 1: 'Warning', 2: 'Error'}

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
        print(levels[level],': ', msg)
    else:
        print('Invalid debug level {} for message "{}". Has to be one of {}'.format(level,msg,levels))

if __name__=='__main__':
    debug(1, 'some ', ' more ', 23)
    debug(999, 'A message')