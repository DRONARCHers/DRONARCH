import bundler,os
from collections import OrderedDict
from debug import debug

import helpers

__author__ = 'niclas'

def start_bundler(imgs_file, match_file, options_file, output_file, output_dir, img_dir, use_old_data= False, orig_imgs=None, imgs=None, vid_imgs=None, focal_length=2.45):
    """
    Starts the bundler pipline.
    To run the pipline with the old data, no imgs and vid_imgs lists have to be provided.

    :param imgs: List of image pathes
    :param vid_imgs: List of image pathes for the images retreived from videos
    :param focal_length: focal length of the camera. Assuming it is a fixed lens camera. If non is declared the value for the Parrot AR 2.0 is used (for historical reason)
    :return:
    """

    #Change to bundler dir
    dir = os.getcwd()
    os.chdir(output_dir)

    if not use_old_data and not imgs==None and not vid_imgs==None: #The old data is not reused and the required lists are defined
        #create dictionaries with ImageName:FocalLenght
        debug(0, 'Start bundler pipline using new images.')
        helpers.timestamp()


        #Use OrderedDictionary to keep order of frames
        vid_imgs_dict =OrderedDict()
        for frame in vid_imgs:
            vid_imgs_dict[frame] = focal_length

        imgs_dict = {}
        # If there are single images, extract focal length
        if len(imgs)>0:
            orig_imgs = [dir+'/'+img for img in orig_imgs]
            print(orig_imgs)
            orig_imgs_dict = bundler.extract_focal_length(orig_imgs)
            for key, value in orig_imgs_dict.items():
                #get path of resized/copied img and create key entry
                key = img_dir+helpers.get_filename_from_path(key)
                imgs_dict[key] = value


        print(imgs_dict)
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
    #start bundler
    return_state = bundler.bundler(image_list=imgs_file,
            options_file=options_file,
            verbose=True,
            match_table=match_file,
            output=helpers.get_filename_from_path(output_file),
            output_all="bundle_",
            output_dir=output_dir,
            variable_focal_length=True,
            use_focal_estimate=True,
            constrain_focal=True,
            constrain_focal_weight=0.0001,
            estimate_distortion=True,
            run_bundle=True)

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