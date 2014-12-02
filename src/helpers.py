import glob,re,os,shutil,subprocess,debug,time

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

        return files

def move_command(path1, path2):
    """
    Moves a file from path1 to path2.
    Should work on all OS
    :param path1:
    :param path2:
    :return:
    """
    print('Execute: Move from ', path1, ' to ', path2)
    try:
        shutil.move(path1, path2)
    except shutil.Error:
        print('File ', path2, 'already exists. I will try overwrite it. This will fail if it is a directory.')
        os.remove(path2)
        shutil.move(path1, path2)


def execute_command(command):
    """
    Executes the command as a shell command
    :param command:
    :return:
    """
    #TODO: Do some security check on the command. Otherwise this is a huge security issue
    print('\tExecute: ', command)
    subprocess.call(command, shell=True)

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
    return time.time()-start_time
def timestamp():
    debug.debug(0,'Time elapsed since start: {:.2}sec'.format(elapsed_time()))