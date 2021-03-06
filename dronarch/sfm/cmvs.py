__author__ = 'niclas'

from dronarch.helpers.helpers import debug, execute_command, get_files_with_ending
from multiprocessing import cpu_count
import os

def run_cmvs(cmvs_bin_folder, pmvs_temp_dir, bundler_out_file, img_size, level=1, csize=1, threshold=0.7, wsize=20, minImageNum=3):

    #change to pmvs dir
    dir = os.getcwd()
    os.chdir(pmvs_temp_dir)


    #copy bundle.out file, because cmvs does not us the file if the name and path is different than ./bundler.rd.out
    command = 'cp '+bundler_out_file+' bundle.rd.out'
    execute_command(command)
    no_clusers = compute_no_clusters(img_size=img_size, level=level, csize=csize, threshold=threshold, wsize=wsize, minImageNum=minImageNum)

    debug(0, 'Starting CMVS using {} clusters'.format(no_clusers))

    #make cmvs calls
    command = cmvs_bin_folder+'cmvs ./ '+str(no_clusers) +' '+str(7)
    execute_command(command)



    debug(0, 'GenOptions')
    CPU = cpu_count()

    command = cmvs_bin_folder+'genOption ./ '+' '.join([str(level), str(csize), str(threshold), str(wsize), str(minImageNum), str(CPU)])
    execute_command(command)

    #change back to previous dir
    os.chdir(dir)

def compute_no_clusters(img_size,  level, csize, threshold, wsize, minImageNum, max_memory=48):
    threshold = 1

    no_pixels = img_size[0]*img_size[1]/4**level
    no_pixels_weight = 1/float(no_pixels)
    no_clusters = 1*10e9
    no_clusters = no_clusters*no_pixels_weight/wsize**2
    no_clusters = no_clusters * max_memory
    no_clusters = int(no_clusters)

    # if  no_clusters<threshold:
	 #    no_clusters=threshold
    return no_clusters

if __name__=='__main__':
    print '{}'.format(compute_no_clusters(img_size=(1998,3000), level=0, csize=1, threshold=0.75, wsize=32, minImageNum=3))
