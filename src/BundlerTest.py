from __future__ import print_function
__author__ = 'niclas'
"""
Messy script to try out bundler, cmvs, pmvs pipline
"""

import bundler, subprocess, glob, shutil, os, cv2


matches_file = 'matches'
imgs_file = 'imgs.txt'
bundler_bin_folder = '/home/niclas/code/dronarch/bundler_sfm/bin/'
cmvs_bin_folder = '/home/niclas/code/dronarch/vsfm/cmvs/program/main/'
pmvs_bin_folder = '/home/niclas/code/dronarch/vsfm/pmvs-2/program/main/'
from_video = True



def imgs_from_video():


def write_file(dicti, filename):
    with open(filename, 'w') as fp:
            for image,value in dicti.items():
                if value == None: fp.write(image + '\n')
                else: fp.write(' '.join([image, '0', str(value), '\n']))
            image_list_file = fp.name
    return image_list_file


def bundler_stuff():
    #load imgs
    imgs =  bundler.get_images()

    #get dictionary img:focal_length
    img_f_dict = bundler.extract_focal_length(imgs)

    #write them to a file. This will be used by bundler and meshlab
    write_file(img_f_dict, imgs_file)

    #get feature points
    keys = bundler.sift_images(imgs,verbose=True, parallel=False) #parallel=True has lead to a system crash!!!
    # keys = ['./sm-14.key', './sm-13.key', './sm-6.key', './sm-7.key', './sm-10.key', './sm-8.key', './sm-5.key', './sm-0.key', './sm-4.key', './sm-11.key', './sm-3.key', './sm-1.key', './sm-9.key', './sm-2.key', './sm-12.key']


    bundler.match_images(keys, matches_file, verbose=True)



    bundler.bundler(image_list=imgs_file,
                options_file="options.txt",
                verbose=True,
                match_table=matches_file,
                output="bundle.out",
                output_all="bundle_",
                output_dir="bundle",
                variable_focal_length=True,
                use_focal_estimate=True,
                constrain_focal=True,
                constrain_focal_weight=0.0001,
                estimate_distortion=True,
                run_bundle=True)

def execute_command(command):
    """
    Executes the command as a shell command
    :param command:
    :return:
    """
    #TODO: Do some security check on the command. Otherwise this is a huge security issue
    print('\tExecute: ', command)
    subprocess.call(command, shell=True)

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

def bundler2pmvs_stuff():
    #call Bundle2PMVS to prepare files for PMVS
    command = bundler_bin_folder+'Bundle2PMVS '+imgs_file+' '+'./bundle/bundle.out pmvs'
    execute_command(command)

    #call RadialUndistort to undistort images.
    #TODO: remove this if camera calibratioon and img undistortion has been implemented
    command = bundler_bin_folder+'/RadialUndistort '+imgs_file+' ./bundle/bundle.out ./pmvs/'
    execute_command(command)

    #move, rename, cleanup according to prep_pmvs.sh
    execute_command('mkdir -p ./pmvs/txt/ ./pmvs/visualize/ ./pmvs/models/')

    file_nr = 0
    for img in glob.glob("./pmvs/*.jpg"):
        num_str = '{:08d}'.format(file_nr)
        move_command(img,'pmvs/visualize/'+num_str+'.jpg')
        move_command('pmvs/'+num_str+'.txt','pmvs/txt/'+num_str+'.txt') #This assumes, that each .txt file correspondes to a .jpg file. I'm not certain if this is true
        file_nr = file_nr+1

    #some last command from the prep_pmvs.sh
    command = bundler_bin_folder+'Bundle2Vis pmvs/bundle.rd.out pmvs/vis.dat'
    execute_command(command)

    command = cmvs_bin_folder+'genOption pmvs/'
    execute_command(command)


def cmvs_stuff():

    #make cmvs calls
    command = cmvs_bin_folder+'cmvs pmvs/ maximage=100 CPU=8'
    execute_command(command)

def pmvs_stuff():
    os.chdir('./pmvs')
    command = pmvs_bin_folder+'pmvs2 ./ pmvs_options.txt'
    execute_command(command)
    os.chdir('./../')
if from_video:
    imgs_from_video()
    bundler_stuff()
# bundler2pmvs_stuff()
# cmvs_stuff()
# pmvs_stuff()

