import glob, re, os, shutil, subprocess, time
from datetime import datetime
from debug import debug

__author__ = 'niclas'

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
    global start_time
    start_time = time.time()
def elapsed_time():
    """
    :return: Elapsed time since start in seconds
    """
    return time.time()-start_time

def timestamp():
    debug(0,'Time elapsed since start: {:.2f}sec'.format(elapsed_time()))

def send_mail(message):
    try:
        import mail
        ts = time.time()
        time_stamp = datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        msg =  'DRONARCH is notifiying you at {} and is telling you: '.format(time_stamp)
        msg = msg +message
        subject = 'Automatic DRONARCH notification'
        mail.send_mail(to_adr='scheuing@students.unibe.ch', subject=subject, msg_content=msg)
    except ImportError:
        debug(1, 'Could not send email. Probably the email script is not available. Ignore this if you are not developer')
if __name__=='__main__':
    send_mail(message='Testing the mail implementation in helpers script of DRONARCH')