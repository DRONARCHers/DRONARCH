__author__ = 'niclas'

from dronarch.helpers.helpers import debug, execute_command
from multiprocessing import cpu_count
import os

def run_cmvs(cmvs_bin_folder, pmvs_temp_dir, bundler_out_file, no_clusers=50, level=1, csize=1, threshold=0.7, wsize=20, minImageNum=3):

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
    CPU = cpu_count()

    command = cmvs_bin_folder+'genOption ./ '+' '.join([str(level), str(csize), str(threshold), str(wsize), str(minImageNum), str(CPU)])
    execute_command(command)

    #change back to previous dir
    os.chdir(dir)