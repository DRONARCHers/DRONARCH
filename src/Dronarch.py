import os, sys, video2image,re,glob,shutil
from debug import debug

__author__ = 'niclas'


class Dronarch:
    """
    The main class. Contains all the coordination and parameters.

    Attributes:
        config_file         The location of the config file
        bundler_bin_dir     The location of the bundler binaries
        cmvs_bin_dir        The location of the CMVS binaries
        pmvs_bin_dir        The location of the PMVS binaries
    """
    #Attributes loaded from config file
    config_file = './../config/dronarch.cfg'
    bundler_bin_dir =''
    cmvs_bin_dir = ''
    pmvs_bin_dir = ''

    #Hardcoded Attributes
    #TODO: Should they be in the config file as well?
    vid_dest_dir = '../vid_imgs/'
    img_dir = '../imgs/'
    video_formats = ['avi','mpeg']
    img_formats = ['jpeg']

    #TODO: ALL DIRECTORIES (that must be created and removes ) HAVE TO BE IN THIS LIST.
    dirs = [vid_dest_dir]


    def __init__(self):
        entries = self.read_config_file(self.config_file)
        self.parse_config_dict(entries)
        if not self.check_attriburtes():
            sys.exit('Config file is incomplete. '+self.config_file)
        self.make_dirs()

    #using with .. as .. guaranties to clean up in the end
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.remove_dirs()
        pass

    def read_config_file(self, config_file):
        """
        Parses a config file and returns a dictionary of its entries.
        The file can contain comments (# comment) and  empty lines.
        The assignment has to be like this: key = value
        :param config_file:
        :return:
        """

        #check whether config file exists. If not exit
        if not os.path.isfile(config_file):
            sys.exit('No valid config file specified. Cannot find: '+config_file)

        with open(config_file, 'r') as file:
            entries = {}
            for line in file:
                line = line.strip()
                if not line.startswith('#') and not line=='': #check for comments and empty lines
                    try:
                        (key, value) = line.split('=')
                        entries[key.strip()] = value.strip()
                    except ValueError: #If line contains more than one =
                        debug(2, 'Invalid cfg file: ', config_file)
                        break
        return entries



    def parse_config_dict(self, entries):
        """
        Parses a dictionary containing the vales from the config file
        This has to be updated if new values are added to the file
        :param entries:
        :return:
        """
        for key,value in entries.items():
            if key=='bundler_bin_dir':
                self.bundler_bin_dir = value
            elif key == 'cmvs_bin_dir':
                self.cmvs_bin_dir = value;
            elif key == 'pmvs_bin_dir':
                self.pmvs_bin_dir = value
            else:
                debug(1, 'Unknown entry: ',key, '=', value)


    def check_attriburtes(self):
        """
        Makes sure that the most important attributes are set before starting execution
        :return:
        """
        if self.bundler_bin_dir== '':
            debug(2, 'bundler_bin_dir not initalized. Check config file ', self.config_file)
            return False
        if self.cmvs_bin_dir== '':
            debug(2, 'cmvs_bin_dir not initalized. Check config file ', self.config_file)
            return False
        if self.pmvs_bin_dir== '':
            debug(2, ' pmvs_bin_dir not initalized. Check config file ', self.config_file)
            return False
        else:
            debug(0, 'All attribures could be read from ', self.config_file)
            return True

    def make_dirs(self):
        """
        Creates all directories specified in self.dirs
        :return:
        """
        for dir in self.dirs:
            if not os.path.exists(dir):
                os.makedirs(dir)

    def remove_dirs(self):
        """
        Removes all directories specified in self.dirs
        :return:
        """
        for dir in self.dirs:
            shutil.rmtree(dir)

    def get_files_with_ending(self, folder, endings):
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

    def start_execution(self):
        videos = self.get_files_with_ending(self.img_dir, self.video_formats)
        if len(videos)>0:
            for video in videos:
                video2image.video2image(video, self.vid_dest_dir, 1,100, 3)


with Dronarch() as dron:
    dron.start_execution()