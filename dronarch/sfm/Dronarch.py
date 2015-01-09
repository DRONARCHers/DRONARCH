#standard python imports
import os
import sys
import shutil
#DRONARCH internal
import video2image
from dronarch.helpers import helpers, img_manipulations
from dronarch.helpers.debug import debug
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
    #Attributes loaded from config file
    config_file = '../../config/dronarch.cfg'
    bundler_bin_dir =''
    cmvs_bin_dir = ''
    pmvs_bin_dir = ''
    vid_calib_img_dir = ''
    img_calib_img_dir = ''
    calib_file_path = ''

    #this is only a fallback value. If specified, the value in the dronarch.cfg file will be used
    img_max_size = (2000,2000)

    vid_imgs_per_sec = 2
    vid_start_frame = 5
    vid_no_images = 10000 #*vid_imgs_per_sec

    #Hardcoded Attributes
    #TODO: Should they be in the config file as well?

    #directories
    temp_dir = '../../roaming/'
    orig_img_dir = '../../imgs/'
    temp_dir = os.path.abspath(temp_dir)+'/'
    vid_dest_dir = temp_dir+'vid_imgs/'
    temp_img_dir = temp_dir+'temp_imgs/'
    video_calib_dest_dir = temp_dir+'temp_vid_calib_imgs/'
    img_calib_dest_dir = temp_dir+'temp_img_calib_imgs/'
    bundler_output_dir = temp_dir+'bundler/'
    pmvs_temp_dir = temp_dir+'pmvs/'



    #bundler files
    bundler_img_name_file = bundler_output_dir+'all_imgs.txt'
    bundler_match_file =  bundler_output_dir+'matches'
    bundler_options_file = bundler_output_dir+'options.txt'
    bundler_output_file = bundler_output_dir+'bundle.out'


    #formats
    video_formats = ['avi','mpeg','mov']
    img_formats = ['jpeg','jpg','JPG','JPEG','bmp', 'BMP']

    dron = None


    #TODO: ALL DIRECTORIES (that must be created and removes ) HAVE TO BE IN THIS LIST.
    dirs = [temp_dir,
            vid_dest_dir,
            temp_img_dir,
            bundler_output_dir,
            pmvs_temp_dir,
            img_calib_dest_dir,
            video_calib_dest_dir
            ]


    def __init__(self):
        entries = self.read_config_file(self.config_file)
        self.parse_config_dict(entries)
        if not self.check_attriburtes():
            sys.exit('Config file is incomplete. '+self.config_file)
        self.make_dirs(self.dirs)

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
            elif key == 'vid_imgs_per_sec':
                self.vid_imgs_per_sec = value
            elif key == 'vid_start_frame':
                self.vid_start_frame = value
            elif key =='vid_no_images':
                self.vid_no_images = value
            elif key == 'vid_calib_img_dir':
                self.vid_calib_img_dir = value
            elif key == 'img_calib_img_dir':
                self.img_calib_img_dir = value
            elif key == 'img_max_size':
                self.img_max_size = self.str2Tuple(value, int)
            elif key == 'calib_file_path':
                self.calib_file_path = value
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

    def make_dirs(self, dirs):
        """
        Creates all directories specified in self.dirs
        :return:
        """
        for dir in dirs:
            if not os.path.exists(dir):
                os.makedirs(dir)
        debug(0, 'Created directories: ', dirs)

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
                                                match_radius=64,
                                                init_imgs=(7,8)
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
                     no_clusers=20
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
    test = False
    use_old_data = True
    send_email = True

    if not use_old_data:
        try:
            shutil.rmtree(Dronarch.temp_dir)
        except OSError:
            pass

    dron = Dronarch.get_instance()
    if test:
        import doctest
        doctest.testmod(extraglobs={'dron': dron})
    else:
        helpers.start_stopwatch()
        dron.start_execution(use_old_data=use_old_data, do_calibration=False, do_bundler=False, do_bundler_2_pmvs=False)

        if send_email:
            t = helpers.elapsed_time()
            t = helpers.time_string(t)
            helpers.send_mail('Bundler finished. Took {} to complete'.format(t))
