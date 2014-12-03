__author__ = 'niclas'

from helpers import debug, execute_command
import os

def run_cmvs(cmvs_bin_folder, pmvs_temp_dir, no_clusers=50):

    #change to pmvs dir
    dir = os.getcwd()
    os.chdir(pmvs_temp_dir)


    debug(0, 'Starting CMVS')
    #make cmvs calls
    command = cmvs_bin_folder+'cmvs ./ '+str(no_clusers) +' '+str(7)
    execute_command(command)

    debug(0, 'GenOptions')
    command = cmvs_bin_folder+'genOption ./'
    execute_command(command)

    #change back to previous dir
    os.chdir(dir)