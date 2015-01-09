"""
Takes care of parallel execution of python functions.
"""
__author__ = 'niclas'

from multiprocessing import Pool, cpu_count

def parallel_exe(function, data, max_threads=None):
    """
    Execute function in parallel with given data
    :param function: Function to be called. This should take one element of data as argument
    :param data: A list of data. Each call of function will be passed ONE element of this list
    e.g.: data= [(1,'a'),(2,'b'),(3,'v')] ==> def function(tupple_of_data): where e.g. tupple_of_data=(1,'a')
    :param max_threads: If this number is less or equal to the maximum number of CPU threads, no more than max_threads threads will be used,
    This is especially useful if a task requires lots of memory and the memory usage can be limited by using less threads.
    :return:
    """
    #get the number of logical processors
    no_processors = cpu_count()

    #make sure at least on thread is not used
    if max_threads == None or max_threads>=no_processors:
        max_threads = no_processors-1

    pool = Pool(processes=max_threads)
    ret = pool.map(function, data)

    return ret
