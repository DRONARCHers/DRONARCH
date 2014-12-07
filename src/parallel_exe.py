from multiprocessing import Pool, cpu_count
__author__ = 'niclas'

def parallel_exe(function, data, max_threads=None):
    #get the number of logical processors
    no_processors = cpu_count()

    #make sure at least on thread is not used
    if max_threads == None or max_threads>=no_processors:
        max_threads = no_processors-1

    pool = Pool(processes=max_threads)
    ret = pool.map(function, data)

    return ret

# def func(names, letter):
#     new_names = []
#     for i in names:
#         new_names.append(i+letter)
#     return True

# names = ['a','b','c','d','e','f','g','h','i','j','k','l','m']
# print func(names)
# ret =  parallel_exe(func, names, max_threads=2)
# print ret
