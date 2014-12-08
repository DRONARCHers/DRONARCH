from __future__ import print_function
import sys

__author__ = 'niclas'

levels = {0:'Debug', 1: 'Warning', 2: 'Error'}

def debug(level,*message):
    """
    All Debug and Error messages should be sent to this function.
    This allows to modify and redirect the messages.
    :param message:
    :param level:
    :return:
    """
    if level in levels.keys():
        print(levels[level],': ', ''.join([str(arg) for arg in message ]))
    else:
        print('Invalid debug level ',level, 'has to be one of ', levels)

# debug(1, 'some ', ' more ', 23)