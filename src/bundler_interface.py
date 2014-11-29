import bundler
from debug import debug
from helpers import get_filename_from_path

__author__ = 'niclas'

def start_bundler(imgs_file, match_file, options_file, output_file, output_dir, use_old_data= False, imgs=None, vid_imgs=None, focal_length=2.45):
    """
    Starts the bundler pipline.
    To run the pipline with the old data, no imgs and vid_imgs lists have to be provided.

    :param imgs: List of image pathes
    :param vid_imgs: List of image pathes for the images retreived from videos
    :param focal_length: focal length of the camera. Assuming it is a fixed lens camera. If non is declared the value for the Parrot AR 2.0 is used (for historical reason)
    :return:
    """

    #
    if not use_old_data and not imgs==None and not vid_imgs==None: #The old data is not reused and the required lists are defined
        #create dictionaries with ImageName:FocalLenght
        debug(0, 'Start bundler pipline using new images.')
        vid_imgs_dict = {frame:focal_length for frame in vid_imgs}

        # If there are single images, extract focal length
        if len(imgs)>0:
            imgs_dict = bundler.extract_focal_length(imgs)
        else:
            imgs_dict = {}

        #merge the two dictionaries
        imgs_dict.update(vid_imgs_dict)

        #write all image pathes to a file. This will be used by bundler and meshlab
        write_file(imgs_dict, imgs_file)

        #get feature points
        keys = bundler.sift_images(imgs_dict.keys(), verbose=True, parallel=False) #parallel=True has lead to a system crash!!!

        #match features
        bundler.match_images(keys, match_file, verbose=True)

    elif not use_old_data: #Illegal state
        debug(2, 'Paramater imgs or vid_imgs are None, but use_old_data is False. Check if the parameters have been defined.')
    else:
        debug(0.,'Start bundler pipline using the data of the last run.')

    a = imgs_file
    b = get_filename_from_path(options_file)
    d = get_filename_from_path(output_file)
    bundler.bundler(image_list=imgs_file,
                    options_file=get_filename_from_path(options_file),
                    verbose=True,
                    match_table=match_file,
                    output=get_filename_from_path(output_file),
                    output_all="bundle_",
                    output_dir='bundle',
                    variable_focal_length=True,
                    use_focal_estimate=True,
                    constrain_focal=True,
                    constrain_focal_weight=0.0001,
                    estimate_distortion=True,
                    run_bundle=True)
    debug(0,'Bundler pipline is finished.')

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
    return image_list_file