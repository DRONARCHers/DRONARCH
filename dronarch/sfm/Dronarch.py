
#standard python imports
import os
import sys
import shutil
#DRONARCH internal
import video2image
from dronarch.helpers import helpers, img_manipulations
from dronarch.helpers.debug import debug, clear_log_file, set_logfiles
from bundler_interface import start_bundler
from bundler2pmvs import run_bundler2pmvs
from pmvs import run_pmvs
from cmvs import run_cmvs
from camera_calibration import calibrate_two_times

#external imports


__author__ = 'niclas'


class Dronarch:
    """
    The main class. Contains all the coordination and parameters.

    Attributes:
        config_file         The location of the config file
        bundler_bin_dir     The location of the bundler binaries
        cmvs_bin_dir        The location of the CMVS binaries
        pmvs_bin_dir        The location of the PMVS binaries

        vid_imgs_per_sec    How many images should be extracted per second from a video
        vid_start_frame     Which frame to start extracting images from a video with
        vid_no_images       How many images should be extracted from a video in total
    """

    dron = None
    config_file = '../../config/dronarch.cfg'

    def __init__(self):
        #Attributes loaded from config file
        self.base_dir = ''
        self.bundler_bin_dir =''
        self.cmvs_bin_dir = ''
        self.pmvs_bin_dir = ''
        self.vid_calib_img_dir = ''
        self.img_calib_img_dir = ''
        self.calib_file_path = ''

        #this is only a fallback value. If specified, the value in the dronarch.cfg file will be used
        self.img_max_size = (2000,2000)

        #video params
        self.vid_imgs_per_sec = None
        self.vid_start_frame = None
        self.vid_no_images = None

        #Bundler stuff
        self.bundler_match_radius=None
        self.bundler_init_imgs = None

        #PMVS stuff
        # These are only a fallback value. If specified, the value in the dronarch.cfg file will be used
        self.pmvs_no_clusters = None
        self.pmvs_level=None
        self.pmvs_csize=None
        self.pmvs_threshold=None
        self.pmvs_wsize=None
        self.pmvs_minImageNum=None

        entries = self.read_config_file(self.config_file)
        self.parse_config_dict(entries)
        if not self.check_attriburtes():
            sys.exit('Config file is incomplete. '+self.config_file)


        #Hardcoded Attributes
        #TODO: Should they be in the config file as well?

        #directories
        self.archiv = self.base_dir+['old_roamings']
        self.temp_dir = self.base_dir+['roaming']
        self.orig_img_dir = self.base_dir+['imgs']
        self.vid_dest_dir = self.temp_dir+['vid_imgs']
        self.temp_img_dir = self.temp_dir+['temp_imgs']
        self.video_calib_dest_dir = self.temp_dir+['temp_vid_calib_imgs']
        self.img_calib_dest_dir = self.temp_dir+['temp_img_calib_imgs']
        self.bundler_output_dir = self.temp_dir+['bundler']
        self.pmvs_temp_dir = self.temp_dir+['pmvs']


        #bundler files
        self.bundler_img_name_file = self.bundler_output_dir+['all_imgs.txt']
        self.bundler_match_file =  self.bundler_output_dir+['matches']
        self.bundler_options_file = self.bundler_output_dir+['options.txt']
        self.bundler_output_file = self.bundler_output_dir+['bundle.out']


        #formats
        self.video_formats = ['avi','mpeg','mov']
        self.img_formats = ['jpeg','jpg','JPG','JPEG','bmp', 'BMP']

        #TODO: ALL DIRECTORIES (that must be created and removes ) HAVE TO BE IN THIS LIST.
        self.dirs = [self.temp_dir,
                self.vid_dest_dir,
                self.temp_img_dir,
                self.bundler_output_dir,
                self.pmvs_temp_dir,
                self.img_calib_dest_dir,
                self.video_calib_dest_dir
                ]

        self.make_dirs(self.dirs)

        #logfiles
        self.logfiles = [self.temp_dir+['dronarch.log'], self.base_dir+['config','dronarch.log'] ]
        set_logfiles(helpers.express_paths(self.logfiles))

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
            if key == 'base_dir':
                self.base_dir= self.str_2_path(value)
            elif key=='bundler_bin_dir':
                self.bundler_bin_dir = self.str_2_path(value)
            elif key == 'cmvs_bin_dir':
                self.cmvs_bin_dir = self.str_2_path(value)
            elif key == 'pmvs_bin_dir':
                self.pmvs_bin_dir = self.str_2_path(value)
            elif key == 'vid_imgs_per_sec':
                self.vid_imgs_per_sec = value
            elif key == 'vid_start_frame':
                self.vid_start_frame = value
            elif key =='vid_no_images':
                self.vid_no_images = value
            elif key == 'vid_calib_img_dir':
                self.vid_calib_img_dir = self.str_2_path(value)
            elif key == 'img_calib_img_dir':
                self.img_calib_img_dir = self.str_2_path(value)
            elif key == 'img_max_size':
                self.img_max_size = self.str2Tuple(value, int)
            elif key == 'calib_file_path':
                self.calib_file_path = value
            elif key == 'pmvs_no_clusters':
                self.pmvs_no_clusters = int(value)
            elif key == 'pmvs_level':
                self.pmvs_level = int(value)
            elif key == 'pmvs_csize':
                self.pmvs_csize = int(value)
            elif key == 'pmvs_threshold':
                self.pmvs_threshold = float(value)
            elif key == 'pmvs_wsize':
                self.pmvs_wsize = int(value)
            elif key == 'pmvs_minImageNum':
                self.pmvs_minImageNum = int(value)
            elif key == 'bundler_match_radius':
                self.bundler_match_radius = int(value)
            elif key == 'bundler_init_imgs':
                self.bundler_init_imgs = self.str2Tuple(value, int)
            elif key == 'do_bundler':
                self.do_bundler= self.str2Bool(value)
            elif key == 'do_calibration':
                self.do_calibration= self.str2Bool(value)
            elif key == 'do_bundler_2_pmvs':
                self.do_bundler_2_pmvs= self.str2Bool(value)
            elif key == 'do_pmvs':
                self.do_pmvs= self.str2Bool(value)
            elif key == 'use_old_data':
                self.use_old_data= self.str2Bool(value)
            elif key == 'send_email':
                self.send_email = self.str2Bool(value)
            elif key == 'vid_imgs_per_sec':
                self.vid_imgs_per_sec  = int(value)
            elif key == 'vid_start_frame':
                 self.vid_start_frame= int(value)
            elif key == 'vid_no_images':
                 self.vid_no_images  = int(value)
            else:
                debug(1, 'Unknown entry: ',key, '=', value)

    def str_2_path(self, string):
        """
        Splits a path separated with / or \ into a list of its components
        :param string:
        :return:
        >>> dron.str_2_path(r'c:\aba\buu')
        ['c:', 'aba', 'buu']
        >>> dron.str_2_path('/home/user/blub.md')
        ['', 'home', 'user', 'blub.md']
        """
        return helpers.split_path(string)

    def check_attriburtes(self):
        """
        Makes sure that the most important attributes are set before starting execution
        :return:
        """
        if self.base_dir=='':
            debug(2, 'base_dir not initalized. Check config file ', self.config_file)
        if self.bundler_bin_dir== '':
            debug(2, 'bundler_bin_dir not initalized. Check config file ', self.config_file)
            return False
        if self.cmvs_bin_dir== '':
            debug(2, 'cmvs_bin_dir not initalized. Check config file ', self.config_file)
            return False
        if self.pmvs_bin_dir== '':
            debug(2, ' pmvs_bin_dir not initalized. Check config file ', self.config_file)
            return False
        if self.pmvs_minImageNum==None or self.pmvs_wsize==None or self.pmvs_threshold==None or self.pmvs_csize==None or self.pmvs_level==None or self.pmvs_no_clusters==None:
            debug(2, 'PMVS parameters pmvs_no_clusters, pmvs_level, pmvs_csize, pmvs_threshold, pmvs_wsize or pmvs_minImageNum not specified. Check config file ', self.config_file)
            return False
        if self.bundler_init_imgs == None or self.bundler_match_radius == None:
            debug(2, 'Bundler parameters bundler_init_imgs or bundler_match_radius not specified. Check config file ', self.config_file)
            return False
        if self.do_bundler==None or self.do_bundler_2_pmvs==None or self.do_calibration==None or self.do_pmvs==None or self.use_old_data==None:
            debug(2, 'Run configuration parameters do_bundler,do_bundler_2_pmvs, do_calibration, do_pmvs or use_old_data not specified. Check config file ', self.config_file)
            return False
        if self.vid_imgs_per_sec == None or self.vid_no_images == None or self.vid_start_frame == None:
            debug(2, 'Video parameters vid_imgs_per_sec, vid_no_images or vid_start_frame not specified. Check config file ', self.config_file)
            return False
        debug(0, 'All attribures could be read from ', self.config_file)
        return True

    def make_dirs(self, dirs):
        """
        Creates all directories specified in self.dirs
        :return:
        """
        for dir in dirs:
            if not os.path.exists(helpers.express_path(dir)):
                os.makedirs(helpers.express_path(dir))
        debug(0, 'Created directories: ', helpers.express_paths(dirs))

    def str2Tuple(self, string, data_type):
        """
        Parses a string into a tuple
        :param string:
        :return:
        >>> dron.str2Tuple('(1,2,3)', int)
        (1, 2, 3)
        """
        string.strip()
        if string[0] == '(':
            string = string[1:]
        if string[-1] == ')':
            string = string[:-1]

        string = string.split(',')
        for i in range(len(string)):
            string[i] = data_type(string[i])
        tup = tuple(string)
        return tup

    def str2Bool(self, string):
        if string=='1':
            return True
        elif string=='0':
            return False
        else:
            debug(2, 'Parsing string "{}" to boolean failed'.format(string))
            return None

    def start_execution(self, use_old_data=False, do_calibration=True, do_bundler=True, do_bundler_2_pmvs=True, do_pmvs=True):
        if not use_old_data:
            #create images from videos and store them
            video_imgs,vid_img_scale, vid_img_size = video2image.check_and_extract_all_videos(src_dir=self.orig_img_dir,
                                                                  dest_dir=self.vid_dest_dir,
                                                                  video_formats=self.video_formats,
                                                                  image_formats=self.img_formats,
                                                                  imgs_per_sec=self.vid_imgs_per_sec,
                                                                  start_frame=self.vid_start_frame,
                                                                  no_images=self.vid_no_images,
                                                                  max_size=self.img_max_size)


            #copy single images to temp dictionary and resize if needed
            imgs,orig_imgs,img_scale,img_size  = img_manipulations.check_and_resize_all(src_dir=self.orig_img_dir,
                                                          dest_dir=self.temp_img_dir,
                                                          max_size=self.img_max_size,
                                                          formats=self.img_formats)
            if do_calibration:
                if len(video_imgs)>0:
                    #calibrate and undistort images from videos
                    calibrate_two_times(calib_img_dir=self.vid_calib_img_dir,
                                        img_dir=self.vid_dest_dir,
                                        dest_dir=self.vid_dest_dir,
                                        calib_dest_dir=self.video_calib_dest_dir,
                                        calib_file_path=self.calib_file_path,
                                        img_endings=self.img_formats,
                                        max_size=vid_img_size,
                                        crop=True
                                        )
                if len(imgs)>0:
                    #calibrate and undistort singel images
                    calibrate_two_times(calib_img_dir=self.img_calib_img_dir,
                                        img_dir=self.temp_img_dir,
                                        dest_dir=self.temp_img_dir,
                                        calib_dest_dir=self.img_calib_dest_dir,
                                        calib_file_path=self.calib_file_path,
                                        img_endings=self.img_formats,
                                        max_size=img_size
                                        )
        else:
            imgs = None
            video_imgs = None
            orig_imgs = None

        if do_bundler:
            #start bundler pipline
            return_state_bundler = start_bundler(imgs_file=self.bundler_img_name_file,
                                                match_file=self.bundler_match_file,
                                                options_file=self.bundler_options_file,
                                                output_file=self.bundler_output_file,
                                                output_dir=self.bundler_output_dir,
                                                img_dir= self.temp_img_dir,
                                                bundler_bin_dir=self.bundler_bin_dir,
                                                calib_file_path=self.calib_file_path,
                                                imgs=imgs,
                                                orig_imgs=orig_imgs,
                                                vid_imgs=video_imgs,
                                                use_old_data=use_old_data,
                                                parallel=True,
                                                match_radius=self.bundler_match_radius,
                                                init_imgs=self.bundler_init_imgs
                                                )
            if not return_state_bundler == 0:
                debug(2, 'Bundler finished with error code ', return_state_bundler)
                exit(return_state_bundler)

        if do_pmvs:
            if do_bundler_2_pmvs:
                run_bundler2pmvs(bundler_bin_folder=self.bundler_bin_dir,
                                 pmvs_temp_dir=self.pmvs_temp_dir,
                                 bundler_image_file=self.bundler_img_name_file,
                                 bundler_out_file=self.bundler_output_file
                )
            run_cmvs(cmvs_bin_folder=self.cmvs_bin_dir,
                     pmvs_temp_dir=self.pmvs_temp_dir,
                     bundler_out_file=self.bundler_output_file,
                     no_clusers=self.pmvs_no_clusters,
                     level=self.pmvs_level,
                     csize=self.pmvs_csize,
                     threshold=self.pmvs_threshold,
                     wsize=self.pmvs_wsize,
                     minImageNum=self.pmvs_minImageNum
            )
            run_pmvs(pmvs_bin_folder=self.pmvs_bin_dir,
                     pmvs_temp_dir=self.pmvs_temp_dir,
            )
        helpers.timestamp()
        debug(0, 'Execution of everything completed.')

    @classmethod
    def get_instance(self):
        if Dronarch.dron == None:
            Dronarch.dron = Dronarch()
        return Dronarch.dron

if __name__ == '__main__':
    if len(sys.argv) > 1:
        try_config_file = sys.argv[1]
        if os.path.isfile(try_config_file):
            debug(0, 'Config file {} found'.format(try_config_file))
            Dronarch.config_file = try_config_file
        else:
            debug(1, 'Config file {} not found. Trying {}'.format(try_config_file, Dronarch.config_file))
            conf_file = Dronarch.config_file
    else:
        debug(1, 'No config file specified. Trying {}'.format(Dronarch.config_file))
        conf_file = Dronarch.config_file
    test = False

    dron = Dronarch.get_instance()

    if not dron.use_old_data:
        try:
            # debug(0, 'Move {} to '.format(dron.temp_dir, dron.archiv))
            # helpers.move_command(path1=dron.temp_dir, path2=dron.archiv)
            clear_log_file()
            debug(0, 'Delete {}'.format(helpers.express_path(dron.temp_dir)))
            shutil.rmtree(helpers.express_path(dron.temp_dir))
            dron.make_dirs(dron.dirs)
            debug(0, 'Create dirs {}'.format(helpers.express_paths(dron.dirs)))
        except OSError:
            pass
    # else:
        # if dron.do_bundler or dron.do_pmvs:
            # new_dir = dron.temp_dir.split('/')
            # new_dir= dron.archiv+new_dir[-1]+helpers.current_data_string_for_filename()
            # debug(0, 'Copy {} to {}'.format(dron.temp_dir, new_dir))
            # shutil.copytree(dron.temp_dir, new_dir)
            # if dron.do_bundler:
            #     debug(0, 'Remove ', dron.pmvs_temp_dir)
            #     shutil.rmtree(dron.pmvs_temp_dir)
    if test:
        from doctest import testfile
        testfile('Dronarch.py', globs={'dron': dron})
    else:
        helpers.start_stopwatch()
        dron.start_execution(use_old_data=dron.use_old_data, do_calibration=dron.do_calibration, do_bundler=dron.do_bundler, do_bundler_2_pmvs=dron.do_bundler_2_pmvs, do_pmvs=dron.do_pmvs)

        if dron.send_email:
            t = helpers.elapsed_time()
            t = helpers.time_string(t)
            helpers.send_mail('Bundler finished. Took {} to complete'.format(t))
