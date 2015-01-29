import os
import sys
import tempfile
import gzip
from collections import OrderedDict
from PIL import Image, ExifTags
from math import tan, pi

from dronarch.helpers.debug import debug
from dronarch.helpers import helpers
from dronarch.helpers.parallel_exe import parallel_exe
from dronarch.helpers.img_manipulations import get_size
from ccd_width import CCD_WIDTHS

__author__ = 'niclas'

BUNDLER_BIN = None
SIFT_BIN = None
MATCH_BIN = None
ENV = None

def start_bundler(imgs_file,
                  match_file,
                  options_file,
                  output_file,
                  output_dir,
                  img_dir,
                  bundler_bin_dir,
                  calib_file_path,
                  orig_imgs=None,
                  imgs=None,
                  vid_imgs=None,
                  video_fov=93,
                  match_radius=15,
                  init_imgs=(0,5),
                  use_old_data= False,
                  parallel=False
):
    """
    Starts the bundler pipline.
    To run the pipline with the old data, no imgs and vid_imgs lists have to be provided.

    :param imgs: List of image pathes
    :param vid_imgs: List of image pathes for the images retreived from videos
    :return:
    """

    #Change to bundler dir
    dir = os.getcwd()
    os.chdir(helpers.express_path(output_dir))


    #set bundler binary globals
    set_bundler_bins(bundler_bin_dir=bundler_bin_dir)

    #set ENV variable with bundler lib path added
    set_lib_path(bundler_bin_dir=bundler_bin_dir)


    if not use_old_data and not imgs==None and not vid_imgs==None: #The old data is not reused and the required lists are defined
        #create dictionaries with ImageName:FocalLenght
        debug(0, 'Start bundler pipline using new images.')
        helpers.timestamp()

        #Use OrderedDictionary to keep order of frames
        vid_imgs_dict =OrderedDict()

        if len(vid_imgs)>0:
            #calculate focal length for video images
            focal_length = focal_length_for_video(vid_imgs[0], video_fov)

            for frame in vid_imgs:
                vid_imgs_dict[frame] = focal_length


        imgs_dict = OrderedDict()
        # If there are single images, extract focal length
        if len(imgs)>0:
            # orig_imgs = [dir+'/'+img for img in orig_imgs]
            orig_imgs_dict = extract_focal_length(images=orig_imgs, resized_imgs= imgs)
            for key, value in orig_imgs_dict.items():
                #get path of resized/copied img and create key entry
                key = helpers.split_path(key)
                key = helpers.express_path(img_dir)+os.path.sep+helpers.get_filename_from_path(key)
                imgs_dict[key] = value

        #merge the two dictionaries. This is a bit tricky, since the order should be kept, wich is not the case when using update()
        total_imgs_dict = OrderedDict()
        for key,value in vid_imgs_dict.items():
            total_imgs_dict[key] = value
        for key,value in imgs_dict.items():
            total_imgs_dict[key] = value
        if len(total_imgs_dict.keys()) == 0:
            debug(2, 'Bundler did not get any images. Maybe the images are not in the correct folder.')
            return 1
        #write all image pathes to a file. This will be used by bundler and meshlab
        write_file(total_imgs_dict, imgs_file)

        #get feature points
        debug(0, 'Start feature detection. This might take a while (up to several hours) and slow down your computer.')
        keys = sift_images(images=helpers.split_paths(total_imgs_dict.keys()), verbose=True, parallel=parallel)
        debug(0,'Feature detection done.')
        helpers.timestamp()

        #match features
        debug(0, 'Start matching features.')
        match_images(key_files=keys, matches_file=match_file, radius=match_radius, verbose=True)
        debug(0, 'Matching features completed')
        helpers.timestamp()

    elif not use_old_data: #Illegal state
        debug(2, 'Paramater imgs or vid_imgs are None, but use_old_data is False. Check if the parameters have been defined.')
    else:
        debug(0.,'Start bundler pipline using the data of the last run.')



    #set path for bundler logfile
    bundler_log_file = output_dir+'bundler.log'

    debug(0, 'Start bundle adjustment. This might take a while (up to several hours).')

    #start bundler
    return_state = bundler(
            image_list_file=imgs_file,
            options_file=options_file,
            logfile=bundler_log_file,
            verbose=True,
            match_table=match_file,
            output=helpers.get_filename_from_path(output_file),
            output_all="bundle_",
            output_dir=output_dir,
            variable_focal_length=True,
            # use_focal_estimate=True,
            # constrain_focal=True,
            # constrain_focal_weight=0.0001,
            estimate_distortion=True,
            run_bundle=True,
            # intrinsics=calib_file_path,
            init_pair1=init_imgs[0],
            init_pair2=init_imgs[1]
    )

    debug(0,'Bundler pipline is finished.')
    helpers.timestamp()
    os.chdir(dir)
    return return_state
def write_file(img_dict, file_path):
    """
    Writes content of img_dict in a bundler specific format into a file
    :param img_dict:
    :param file_path:
    :return: The path of the created file
    """
    with open(helpers.express_path(file_path), 'w+') as file:
            for image,focal_length in img_dict.items():
                #use the  format bundler expects
                if focal_length == None:
                    file.write(' '.join([image, '0', '0', '\n']))
                else:
                    file.write(' '.join([image, '0', str(focal_length), '\n']))
            image_list_file = file.name
    debug(0, 'Saved image file: ', helpers.express_path(file_path))
    return image_list_file


def extract_focal_length(images, resized_imgs, verbose=True):
    """Extracts (pixel) focal length from images where available.
    The functions returns a dictionary of image, focal length pairs.
    If no focal length is extracted for an image, the second pair is None.
    """
    ret = OrderedDict()
    for i in range(len(images)):
        image = images[i]
        image_name = helpers.express_path(image)

        new_image = resized_imgs[i]
        new_image_name = helpers.express_path(new_image)
        if verbose:
            debug(0, 'Extracting EXIF tags from image {0}'.format(image_name))

        tags = {}
        with open(image_name, 'rb') as fp:
            img = Image.open(fp)
            if hasattr(img, '_getexif'):
                exifinfo = img._getexif()
                if exifinfo is not None:
                    for tag, value in exifinfo.items():
                        tags[ExifTags.TAGS.get(tag, tag)] = value

        ret[new_image_name] = None

        # Extract Focal Length
        focalN, focalD = tags.get('FocalLength', (0, 1))
        focal_length = float(focalN)/float(focalD)

        # Extract Resolution
        img_width,img_height,img_depth = get_size(new_image)


        if img_width < img_height:
            img_width,img_height = img_height,img_width


        # Extract CCD Width (Prefer Lookup Table)
        make_model = tags.get('Make', '') + ' ' + tags.get('Model', '')
        if CCD_WIDTHS.has_key(make_model.strip()):
            ccd_width = CCD_WIDTHS[make_model.strip()]
        else:
            fplaneN, fplaneD = tags.get('FocalPlaneXResolution', (0, 1))
            if fplaneN != 0:
                ccd_width = 25.4*float(img_width)*float(fplaneD)/float(fplaneN)
                if verbose: debug(0,'Using CCD width from EXIF tags')
            else:
                ccd_width = 0

        if verbose:
            debug(0,'EXIF focal length = {0}mm' .format(focal_length),
                    ', EXIF CCD width = {0}mm'.format(ccd_width),
                    ', EXIF resolution = {0} x {1}'.format(img_width, img_height))
            if ccd_width == 0:
                debug(1,'No CCD width available for camera {0}]'.format(
                    make_model))

        if (img_width==0 or img_height==0 or focalN==0 or ccd_width==0):
            if verbose:
                debug(1,'Could not determine pixel focal length')
            continue

        # Compute Focal Length in Pixels
        ret[new_image_name] = img_width * (focal_length / ccd_width)
        if verbose:
            debug(0,'Focal length (pixels) = {0}'.format(ret[new_image_name]))

    return ret

def calculate_focal_length_in_px(img_width, fov):
    focal_pixel = (float(img_width) * 0.5) / tan(fov * 0.5 * pi/180)
    return focal_pixel

def focal_length_for_video(image, fov):
    width, width, height = get_size(image)
    return calculate_focal_length_in_px(width, fov)


def bundler(image_list_file, options_file, logfile, shell=False, *args, **kwargs):
    """Run bundler, parsing arguments from args and kwargs through.
    For Bundler usage run bundler("--help").

    image_list : File containing list of images.
    options_file : Specify an options file for bundler (optional).
    shell : Enable full shell support for parsing args (default: False).
    """
    def kwargs_bool(b, r):
        if b: return r
        else: return []

    kwargs_dict = {
        'match_table'            : lambda k,v: ['--'+k,v],
        'output'                 : lambda k,v: ['--'+k,v],
        'output_all'             : lambda k,v: ['--'+k,v],
        'output_dir'             : lambda k,v: ['--'+k,v],
        'variable_focal_length'  : lambda k,v: kwargs_bool(v, ['--'+k]),
        'use_focal_estimate'     : lambda k,v: kwargs_bool(v, ['--'+k]),
        'constrain_focal'        : lambda k,v: kwargs_bool(v, ['--'+k]),
        'constrain_focal_weight' : lambda k,v: ['--'+k,str(v)],
        'estimate_distortion'    : lambda k,v: kwargs_bool(v, ['--'+k]),
        'run_bundle'             : lambda k,v: kwargs_bool(v, ['--'+k]),
        'intrinsics'             : lambda k,v : ['--'+k,v],
        'init_pair1'             : lambda k,v : ['--'+k,str(v)],
        'init_pair2'             : lambda k,v : ['--'+k,str(v)]
    }

    str_args = [a for a in args if type(a) == str]
    for k,v in kwargs.items():
        if not kwargs_dict.has_key(k): continue
        str_args.extend(kwargs_dict[k](k,v))

    if len(str_args) != 0 and options_file is not None:
        with open(options_file, 'wb') as fp:
            for o in str_args:
                if o.startswith('--'): fp.write('\n')
                else: fp.write(' ')
                fp.write(o)

    with open(logfile, 'wb') as fp_out:
        if options_file is not None:
            command = [BUNDLER_BIN, image_list_file, "--options_file",
                options_file]
            out = helpers.execute_command(command, shell=shell, env=ENV, stdout=fp_out)
        else:
            debug(2, 'No option file for bundler found')

    return out


def sift_image(image, verbose=False):
    """Extracts SIFT features from a single image.  See sift_images."""




    pgm_filename = image.rsplit('.', 1)[0] + ".pgm"
    key_filename = image.rsplit('.', 1)[0] + ".key"

    # Convert image to PGM format (grayscale)
    with open(image, 'rb') as fp_img:
        image = Image.open(fp_img)
        image.convert('L').save(pgm_filename)

    # Extract SIFT data
    if verbose:
        with open(pgm_filename, 'rb') as fp_in:
            with open(key_filename, 'wb') as fp_out:
                helpers.execute_command(SIFT_BIN, stdin=fp_in, stdout=fp_out)
    else:
        with open(pgm_filename, 'rb') as fp_in:
            with open(key_filename, 'wb') as fp_out:
                with open(os.devnull, 'w') as fp_err:
                    helpers.execute_command(SIFT_BIN, stdin=fp_in, stdout=fp_out, stderr=fp_err)
                    # subprocess.call(SIFT_BIN, stdin=fp_in, stdout=fp_out,
                    #                 stderr=fp_err)

    # Remove pgm file
    os.remove(pgm_filename)

    # GZIP compress key file (and remove)
    with open(key_filename, 'rb') as fp_in:
        with gzip.open(key_filename + ".gz", 'wb') as fp_out:
            fp_out.writelines(fp_in)
    os.remove(key_filename)

    return key_filename


def sift_images(images, verbose=False, parallel=False):
    """Extracts SIFT features from images in 'images'.

    'images' should be a list of file names.  The function creates a
    SIFT compressed key file for each image in 'images' with a '.key.gz'
    extension.  A list of the uncompressed key file names is returned.

    If 'parallel' is True, the function executes SIFT in parallel.
    """
    key_filenames = []

    if parallel:
        #if the images are large, extracting features requires lots of memory. So less threads should be used in parallel
        img_size = get_size(images[0])
        if img_size[0]>3000 or img_size[1]>3000:
            threads=2
        elif img_size[0]>2500 or img_size[1]>2500:
            threads=3
        elif img_size[0]>2000 or img_size[1]>2000:
            threads=4
        else:
            threads = 5
        debug(0, 'Using {} threads for feature detection'.format(threads))
        image_names = helpers.express_paths(images)
        key_filenames = parallel_exe(sift_image, image_names, max_threads=threads)
    else:
        for image in images:
            key_filenames.append(sift_image(image, verbose=verbose))
    return key_filenames


def match_images(key_files, matches_file, radius, verbose=False):
    """Executes KeyMatchFull to match key points in images."""

    time = helpers.time_string(sum([i for i in range(len(key_files))]))
    debug(0, 'Maching of {} images will take between {} and {} (I guess)'.format(len(key_files), helpers.time_string(len(key_files)*30), time))

    with tempfile.NamedTemporaryFile(delete=False) as fp:
        for key in key_files:
            fp.write(key + '\n')
        keys_file = fp.name

    #execute match
    command = ' '.join([MATCH_BIN, keys_file, matches_file, str(radius)])
    if verbose:
        helpers.execute_command(command, env=ENV)
    else:
        with open(os.devnull, 'w') as fp_out:
            helpers.execute_command(command,  stdout=fp_out, env=ENV)

    os.remove(keys_file)


def set_bundler_bins(bundler_bin_dir):
    global BUNDLER_BIN, SIFT_BIN, MATCH_BIN

    if sys.platform == 'win32' or sys.platform == 'cygwin':
        bundler_bin = os.path.join(helpers.express_path(bundler_bin_dir), "Bundler.exe")
        sift_bin = os.path.join(helpers.express_path(bundler_bin_dir), "siftWin32.exe")
        match_bin = os.path.join(helpers.express_path(bundler_bin_dir), "KeyMatchFull.exe")
    else:
        bundler_bin = os.path.join(helpers.express_path(bundler_bin_dir), "bundler")
        sift_bin = os.path.join(helpers.express_path(bundler_bin_dir), "sift")
        match_bin = os.path.join(helpers.express_path(bundler_bin_dir), "KeyMatchFull")

    BUNDLER_BIN = bundler_bin
    SIFT_BIN = sift_bin
    MATCH_BIN = match_bin

def set_lib_path(bundler_bin_dir):
    global ENV

    # Add lib folder to LD_LIBRARY_PATH
    env = dict(os.environ)
    bundler_lib_path = os.path.join(helpers.express_path(bundler_bin_dir), "../lib")

    if env.has_key('LD_LIBRARY_PATH'):
        env['LD_LIBRARY_PATH'] = env['LD_LIBRARY_PATH'] + ':' + bundler_lib_path
    else:
        env['LD_LIBRARY_PATH'] = bundler_lib_path

    ENV = env

