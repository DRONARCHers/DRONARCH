import os
from dronarch.helpers.helpers import execute_command
from dronarch.helpers.debug import debug
from dronarch.helpers.Path import Path
from glob import glob
__author__ = 'niclas'

def run_pmvs(pmvs_bin_folder, pmvs_temp_dir):
    #change to pmvs dir
    dir = os.getcwd()
    os.chdir(pmvs_temp_dir.express())

    debug(0, 'Starting PMVS. This might take a while.')
    option_desc = pmvs_temp_dir.new_app_path('option-*').express()
    option_files = glob(option_desc)
    if len(option_files) == 0:
        os.chdir(dir)
        debug(2, 'No option files for pmvs.')
    else:
        for file in option_files:

            file = Path(file).path_list[-1]
            command = pmvs_bin_folder.new_app_path('pmvs2').express()+' ./ '+file
            execute_command(command)

    #change back to previous dir
    os.chdir(dir)