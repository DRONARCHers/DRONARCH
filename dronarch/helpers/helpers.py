"""
Contains a number of mixed helper functions
"""
__author__ = 'niclas'

import glob, re, os, shutil, subprocess, time, rostopic
from datetime import datetime
from debug import debug



#Used for the timer/timestamp
global start_time


def get_files_with_ending(folder, endings):
    """
    Scan the folder for files with specified ending and return them as a list
    :param folder: The folder to be searched
    :param endings: List containing the endings without the dot. .e.g. ['avi', 'mpeg']
    :return:
    """

    #get all files
    files = glob.glob(folder+'*')

    #compile the most epic regex patterne ever
    pattern = ''.join([''+end+'|' for end in endings])
    #add brackets and remove last pipe
    pattern = '.*\.('+pattern[0:-1]+')$'
    regex = re.compile(pattern, re.IGNORECASE)

    #filter to keep matches only
    files = filter(lambda x: regex.match(x), files)

    #sort images to guarantee deterministic order
    files.sort()

    return files

def move_command(path1, path2):
    """
    Moves a file from path1 to path2.
    Should work on all OS
    :param path1:
    :param path2:
    :return:
    """
    debug(0,'Execute: Move from ', path1, ' to ', path2)
    try:
        shutil.move(path1, path2)
        debug(0,'File moved from ',path1,' to ', path2 )
    except shutil.Error:
        debug(1, 'File ', path2, 'already exists. I will try overwrite it. This will fail if it is a directory.')
        os.remove(path2)
        shutil.move(path1, path2)


def execute_command(command, shell=True, env=None, stdout=None, stdin=None,stderr=None):
    """
    Executes the command as a shell command
    :param command:
    :return:
    """
    #TODO: Do some security check on the command. Otherwise this is a bit of a security issue
    debug(0,'Execute: ', command)

    ret = subprocess.call(command, env=env, stdin=stdin, stdout=stdout, stderr=stderr, shell=shell)
    return ret

def get_filename_from_path(path):
    """
    Takes a path as string and returns the filename without the super directories.
    :param path: Path of file
    :return: Filename
    """
    name = path.split('/')
    name = ''.join(name[-1])
    return name


def start_stopwatch():
    """
    Start the gobal stopwatch
    :return:
    """
    global start_time
    start_time = time.time()

def elapsed_time():
    """
    :return: Elapsed time since start in seconds
    """
    return time.time()-start_time

def timestamp():
    """
    Prints a timestamp to the dronarch.helpers.debug
    :return:
    """
    debug(0,'Time elapsed since start: ', time_string(elapsed_time()) )

def time_string(t):
    """
    Returns a formated string of an integer representing time in seconds
    :param t:
    :return:
    """
    h,h_rem = divmod(t,60*60)
    m,s = divmod(h_rem, 60)
    return '{}h {:02d}m {:02d}s '.format(int(h),int(m),int(s))

def date_string(time):
    return datetime.fromtimestamp(time).strftime('%d.%m.%Y %H:%M:%S')

def current_data_string_for_filename():
    return datetime.fromtimestamp(time.time()).strftime('%d-%m-%Y_%H-%M-%S')

def send_mail(message):
    """
    Send an email to the address specified in the mail.py file to notify that dronarch has completed
    :param message:
    :return:
    """
    try:
        import mail_nick
        ts = time.time()
        time_stamp = date_string(ts)
        msg =  'DRONARCH is notifiying you at {} and is telling you: '.format(time_stamp)
        msg = msg +message
        subject = 'Automatic DRONARCH notification'
        mail_nick.send_dronarch_mail(subject=subject, msg_content=msg)
    except ImportError:
        debug(1, 'Could not send email. Probably the email script is not available. Ignore this if you are not developer')

def ros_core_is_running():
    """
    Check whether roscore is running
    :return:
    """
    try:
        # Checkif rosmaster is running or not.
        rostopic.get_topic_class('/rosout')
        is_rosmaster_running = True
    except rostopic.ROSTopicIOException as e:
        is_rosmaster_running = False
    return is_rosmaster_running



if __name__=='__main__':
    # send_mail(message='Testing the mail implementation in helpers script of DRONARCH')
    start_stopwatch()
    print time_string(elapsed_time())
    print date_string(time.time())