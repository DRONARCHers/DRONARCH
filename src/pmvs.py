import os
from helpers import execute_command, get_filename_from_path
from debug import debug
from glob import glob
__author__ = 'niclas'

def run_pmvs(pmvs_bin_folder, pmvs_temp_dir):
    #change to pmvs dir
    dir = os.getcwd()
    os.chdir(pmvs_temp_dir)

    debug(0, 'Starting PMVS. This might take a while.')
    option_files = glob(pmvs_temp_dir+'option-*')
    for file in option_files:
        command = pmvs_bin_folder+'pmvs2 ./ '+get_filename_from_path(file)
        execute_command(command)

    #change back to previous dir
    os.chdir(dir)