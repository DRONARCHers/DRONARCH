import bundler,os
from img_manipulations import get_size
from collections import OrderedDict
from debug import debug
from PIL import Image, ExifTags
from math import tan, pi
import helpers

__author__ = 'niclas'

def start_bundler(imgs_file, match_file, options_file, output_file, output_dir, img_dir, use_old_data= False, orig_imgs=None, imgs=None, vid_imgs=None,video_fov=93):
    """
    Starts the bundler pipline.
    To run the pipline with the old data, no imgs and vid_imgs lists have to be provided.

    :param imgs: List of image pathes
    :param vid_imgs: List of image pathes for the images retreived from videos
    :return:
    """

    #Change to bundler dir
    dir = os.getcwd()
    os.chdir(output_dir)

    if not use_old_data and not imgs==None and not vid_imgs==None: #The old data is not reused and the required lists are defined
        #create dictionaries with ImageName:FocalLenght
        debug(0, 'Start bundler pipline using new images.')
        helpers.timestamp()

        #calculate focal length for video images
        focal_length = focal_length_for_video(vid_imgs[0], video_fov)

        #Use OrderedDictionary to keep order of frames
        vid_imgs_dict =OrderedDict()
        for frame in vid_imgs:
            vid_imgs_dict[frame] = focal_length


        imgs_dict = {}
        # If there are single images, extract focal length
        if len(imgs)>0:
            orig_imgs = [dir+'/'+img for img in orig_imgs]
            orig_imgs_dict = extract_focal_length(images=orig_imgs, resized_imgs= imgs)
            for key, value in orig_imgs_dict.items():
                #get path of resized/copied img and create key entry
                key = img_dir+helpers.get_filename_from_path(key)
                imgs_dict[key] = value

        #merge the two dictionaries. This is a bit tricky, since the order should be kept, wich is not the case when using update()
        total_imgs_dict = {}#OrderedDict()
        for key,value in vid_imgs_dict.items():
            total_imgs_dict[key] = value
        for key,value in imgs_dict.items():
            total_imgs_dict[key] = value

        #write all image pathes to a file. This will be used by bundler and meshlab
        write_file(total_imgs_dict, imgs_file)

        #get feature points
        debug(0, 'Start feature detection. This might take a while (up to several hours) and slow down your computer.')
        keys = bundler.sift_images(total_imgs_dict.keys(), verbose=True, parallel=True) #parallel=True has lead to a system crash!!!
        debug(0,'Feature detection done.')
        helpers.timestamp()

        #match features
        debug(0, 'Start matching features.')
        bundler.match_images(keys, match_file, verbose=True)
        debug(0, 'Matching features completed')
        helpers.timestamp()

    elif not use_old_data: #Illegal state
        debug(2, 'Paramater imgs or vid_imgs are None, but use_old_data is False. Check if the parameters have been defined.')
    else:
        debug(0.,'Start bundler pipline using the data of the last run.')
        if False:

            keys = ['/home/niclas/code/dronarch/project/roaming/vid_imgs/test2_'+str(i)+'.key' for i in range(0,20)]
            bundler.match_images(keys, match_file, verbose=True)



    debug(0, 'Start bundle adjustment.')
    helpers.timestamp()

    debug(0, 'Start bundle adjustment. This might take a while (up to several hours).')
    # exit(1)
    #start bundler
    return_state = bundler.bundler(image_list=imgs_file,
            options_file=options_file,
            verbose=True,
            match_table=match_file,
            output=helpers.get_filename_from_path(output_file),
            output_all="bundle_",
            output_dir=output_dir,
            variable_focal_length=False,
            # use_focal_estimate=True,
            # constrain_focal=True,
            # constrain_focal_weight=0.0001,
            # estimate_distortion=True,
            run_bundle=True
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
    with open(file_path, 'w+') as file:
            for image,focal_length in img_dict.items():
                #use the  format bundler expects
                if focal_length == None:
                    file.write(image + '\n')
                else:
                    file.write(' '.join([image, '0', str(focal_length), '\n']))
            image_list_file = file.name
    debug(0, 'Saved image file: ', file_path)
    return image_list_file


def extract_focal_length(images, resized_imgs, verbose=True):
    """Extracts (pixel) focal length from images where available.
    The functions returns a dictionary of image, focal length pairs.
    If no focal length is extracted for an image, the second pair is None.
    """
    ret = OrderedDict()
    for i in range(len(images)):
        image = images[i]
        new_image = resized_imgs[i]
        if verbose:
            debug(0, 'Extracting EXIF tags from image {0}'.format(image))

        tags = {}
        with open(image, 'rb') as fp:
            img = Image.open(fp)
            if hasattr(img, '_getexif'):
                exifinfo = img._getexif()
                if exifinfo is not None:
                    for tag, value in exifinfo.items():
                        tags[ExifTags.TAGS.get(tag, tag)] = value

        ret[new_image] = None

        # Extract Focal Length
        focalN, focalD = tags.get('FocalLength', (0, 1))
        focal_length = float(focalN)/float(focalD)

        # Extract Resolution
        img_width,img_height,img_depth = get_size(new_image)


        if img_width < img_height:
            img_width,img_height = img_height,img_width


        # Extract CCD Width (Prefer Lookup Table)
        make_model = tags.get('Make', '') + ' ' + tags.get('Model', '')
        if bundler.CCD_WIDTHS.has_key(make_model.strip()):
            ccd_width = bundler.CCD_WIDTHS[make_model.strip()]
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
        ret[new_image] = img_width * (focal_length / ccd_width)
        if verbose:
            debug(0,'Focal length (pixels) = {0}'.format(ret[new_image]))

    return ret

def calculate_focal_length_in_px(img_width, fov):
    focal_pixel = (float(img_width) * 0.5) / tan(fov * 0.5 * pi/180)
    return focal_pixel

def focal_length_for_video(image, fov):
    width, width, height = get_size(image)
    return calculate_focal_length_in_px(width, fov)

print calculate_focal_length_in_px(720, 93)