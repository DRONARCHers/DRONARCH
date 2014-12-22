__author__ = 'niclas'

from dronarch.helpers.helpers import debug, execute_command, move_command
import os

def run_cmvs(cmvs_bin_folder, pmvs_temp_dir, bundler_out_file, no_clusers=50):

    #change to pmvs dir
    dir = os.getcwd()
    os.chdir(pmvs_temp_dir)

    #copy bundle.out file, because cmvs does not us the file if the name and path is different than ./bundler.rd.out
    command = 'cp '+bundler_out_file+' bundle.rd.out'
    execute_command(command)

    debug(0, 'Starting CMVS')
    #make cmvs calls
    command = cmvs_bin_folder+'cmvs ./ '+str(no_clusers) +' '+str(7)
    execute_command(command)



    debug(0, 'GenOptions')
    command = cmvs_bin_folder+'genOption ./'
    execute_command(command)

    #change back to previous dir
    os.chdir(dir)