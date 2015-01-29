__author__ = 'niclas'
from dronarch.helpers.helpers import move_command,execute_command, get_files_with_ending, timestamp
from dronarch.helpers.debug import debug
import glob,os

def run_bundler2pmvs(bundler_bin_folder, bundler_image_file, bundler_out_file,  pmvs_temp_dir):

     #call Bundle2PMVS to prepare files for PMVS
    debug(0, 'Starting Bundle2PMVS. This might take a while.')
    command = bundler_bin_folder+'Bundle2PMVS '+bundler_image_file+' '+bundler_out_file+' '+pmvs_temp_dir
    execute_command(command)
    timestamp()

    #call RadialUndistort to undistort images.
    # #TODO: No longer needed, calibration is done before using chessboard calibration
    command = bundler_bin_folder+'/RadialUndistort '+bundler_image_file+' '+bundler_out_file+' '+pmvs_temp_dir
    execute_command(command)


    #move, rename, cleanup according to prep_pmvs.sh
    execute_command('mkdir -p '+pmvs_temp_dir+'txt/ '+pmvs_temp_dir+'visualize/ '+pmvs_temp_dir+'models/')

    file_nr = 0
    for img in get_files_with_ending(folder=pmvs_temp_dir, endings=['jpg']):
        num_str = '{:08d}'.format(file_nr)
        move_command(img,pmvs_temp_dir+'visualize/'+num_str+'.jpg')
        move_command(pmvs_temp_dir+num_str+'.txt',pmvs_temp_dir+'txt/'+num_str+'.txt') #This assumes, that each .txt file correspondes to a .jpg file. I'm not certain if this is true
        file_nr = file_nr+1

    #some last command from the prep_pmvs.sh
    command = bundler_bin_folder+'Bundle2Vis '+bundler_out_file+' '+pmvs_temp_dir+'vis.dat'
    execute_command(command)


