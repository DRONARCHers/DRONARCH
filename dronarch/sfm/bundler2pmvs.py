__author__ = 'niclas'
from dronarch.helpers.helpers import move_command,execute_command, get_files_with_ending, timestamp
from dronarch.helpers.debug import debug
import sys,os,shutil

def run_bundler2pmvs(bundler_bin_folder, bundler_image_file, bundler_out_file,  pmvs_temp_dir):

    #create binary command path
    if sys.platform == 'win32' or sys.platform == 'cygwin':
        bundle2pmvs_bin = bundler_bin_folder.new_app_path('Bundle2PMVS.exe')
        radial_undistort_bin = bundler_bin_folder.new_app_path('RadialUndistort.exe')
        bundle2vis_bin = bundler_bin_folder.new_app_path('Bundle2Vis.exe')
    else:
        bundle2pmvs_bin = bundler_bin_folder.new_app_path('Bundle2PMVS')
        radial_undistort_bin = bundler_bin_folder.new_app_path('RadialUndistort')
        bundle2vis_bin = bundler_bin_folder.new_app_path('Bundle2Vis')

    #call Bundle2PMVS to prepare files for PMVS
    debug(0, 'Starting Bundle2PMVS. This might take a while.')
    command = bundle2pmvs_bin.express()+' '+bundler_image_file.express()+' '+bundler_out_file.express()+' '+pmvs_temp_dir.express()
    execute_command(command)
    timestamp()

    #call RadialUndistort to undistort images.
    # #TODO: No longer needed, calibration is done before using chessboard calibration
    command = radial_undistort_bin.express()+' '+bundler_image_file.express()+' '+bundler_out_file.express()+' '+pmvs_temp_dir.express()
    execute_command(command)


    #move, rename, cleanup according to prep_pmvs.sh
    txt_dir = pmvs_temp_dir.new_app_path('txt/')
    vis_dir = pmvs_temp_dir.new_app_path('visualize/')
    model_dir = pmvs_temp_dir.new_app_path('models/')
    dirs = ' '.join([txt_dir.express(), vis_dir.express(), model_dir.express()])

    if os.path.isdir(txt_dir.express()):
        shutil.rmtree(txt_dir.express())
    if os.path.isdir(vis_dir.express()):
        shutil.rmtree(vis_dir.express())
    if os.path.isdir(model_dir.express()):
        shutil.rmtree(model_dir.express())

    execute_command('mkdir '+dirs)

    file_nr = 0
    for orig_jpg in get_files_with_ending(folder=pmvs_temp_dir, endings=['jpg']):
        num_str = '{:08d}'.format(file_nr)
        dest_jpg = pmvs_temp_dir.new_app_path('visualize/'+num_str+'.jpg')
        move_command(orig_jpg, dest_jpg)

        orig_txt = pmvs_temp_dir.new_app_path(num_str+'.txt')
        dest_txt = pmvs_temp_dir.new_app_path('txt/'+num_str+'.txt')
        move_command(orig_txt,dest_txt) #This assumes, that each .txt file correspondes to a .jpg file. I'm not certain if this is true
        file_nr = file_nr+1

    #some last command from the prep_pmvs.sh
    command = bundle2vis_bin.express()+' '+bundler_out_file.express()+' '+pmvs_temp_dir.new_app_path('vis.dat').express()
    execute_command(command)


