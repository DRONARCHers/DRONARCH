import glob,re

__author__ = 'niclas'

def get_files_with_ending(folder, endings):
        """
        Scan the folder for files with specified ending and return them as a list
        :param folder: The folder to be searched
        :param endings: List containing the endings without the dot. .e.g. ['avi', 'mpeg']
        :return:
        """

        #get all files
        files = glob.glob(folder+'*')

        #compile the most epic regex patterne ever
        pattern = ''.join([''+end+'|' for end in endings])
        #add brackets and remove last pipe
        pattern = '.*\.('+pattern[0:-1]+')$'
        regex = re.compile(pattern, re.IGNORECASE)

        #filter to keep matches only
        files = filter(lambda x: regex.match(x), files)

        return files