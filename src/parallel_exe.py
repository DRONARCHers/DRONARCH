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

# class cla:
#     letter = 'L'
#     def func(self,names):
#         new_names = []
#         for i in names:
#             new_names.append(i+self.letter)
#         return new_names
#
# names = ['a','b','c','d','e','f','g','h','i','j','k','l','m']
# c = cla()
# print c.func(names)
# ret =  parallel_exe(cla.func, names, max_threads=2)
#
# print ret
