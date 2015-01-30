__author__ = 'scheuing'
import os

class Path:
    def __init__(self, path=None):
        """
        Creates a new Path object using path if given. path can be a string or a list containing the paths components as strings.
        :param string:
        :param path_list:
        :return:
        """
        if not path == None and isinstance(path, str):
            self.path_list = self.split_path(path)
        elif not path == None and isinstance(path, list):
            self.path_list = path
        else:
            self.path_list = []

    def express(self, separator=None):
        """
        Expresses the Path object as a str using the correct separator (os dependent) if none is given as argument.
        :return:
        >>> path = Path('/home/b/cv.a')
        >>> path.express(separator="/")
        '/home/b/cv.a'
        """

        if separator == None:
            separator = os.path.sep
        path_str = separator.join(self.path_list)
        return path_str
    def get_filename(self):
        """
        Get filename without base dir path
        :return:
        """
        return self.path_list[-1]
    def split_path(self, string):
        """
        Splits string containing a path into a list of its components.
        :param string:
        :return:
        >>> path = Path('a/b')
        >>> path.split_path('/home/user/blub.md')
        ['', 'home', 'user', 'blub.md']
        >>> path.split_path('c:\\ka\\po/zu/')
        ['c:', 'ka', 'po', 'zu']
        """
        path = string.split('/')
        path_list = []
        for ele in path:
            path_list.extend(ele.split('\\'))
        base = path_list[0]
        path_list = [ele for ele in path_list[1:] if not ele.strip() == '' or ele ==None]
        path_list = [base]+path_list
        return path_list

    def append(self, append_path):
        self.path_list.extend(append_path)

    def new_app_path(self, append_path):
        """
        Creates a new Path object based on self but adds the append_path.
        :param append_path:
        :return:
        >>> path = Path('c:\\haha\\ ')
        >>> path2 = path.new_app_path('bla/blub')
        >>> path2 == Path(['c:', 'haha','bla','blub'])
        True
        >>> path2 = path.new_app_path(['bla'])
        >>> path2 == Path(['c:', 'haha','bla'])
        True
        """
        if isinstance(append_path, str):
            append_path = self.split_path(append_path)
        new_path_list = self.path_list+append_path
        path = Path(new_path_list)
        return path

    def __eq__(self, other):
        """
        :param other:
        :return:
        >>> Path('ab/cd') == Path('ab\cd')
        True
        >>> Path() == None
        False
        """
        return isinstance(other, type(self)) and self.path_list == other.path_list

    def __str__(self):
        return self.express()

    @classmethod
    def split_paths(self, paths):
        return [path.split_path() for path in paths]

    @classmethod
    def create_paths(self, pathes):
        """
        Takes a list of pathes (as string or lists) and returns a list os Path objects with the given path value.
        :param pathes:
        :return:

        >>> pathes = Path.create_paths(['/home/du', ['bla', 'blub']])
        >>> pathes[0] == Path('/home/du')
        True
        >>> pathes[1] == Path(['bla', 'blub'])
        True
        """
        path_objs = [Path(path) for path in pathes]
        return path_objs
    @classmethod
    def express_paths(self, paths):
        return [path.express() for path in paths]

if __name__=='__main__':
    import doctest
    doctest.testmod()