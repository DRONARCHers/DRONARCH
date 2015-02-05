__author__ = 'scheuing'


def combinations(liste):
    """
    Returns all possible pairs of items in a list liste
    :param liste:
    :return:
    >>> l = list(range(3))
    >>> l2 = combinations(l)
    >>> [(0,1), (0,2), (1,2)] == l2
    True
    >>> l = ['a','g', 1]
    >>> l2 = combinations(l)
    >>> [('a','g'), ('a',1), ('g',1)] == l2
    True
    """

    l = [(liste[i1], liste[i2]) for i1 in range(len(liste)) for i2 in range(len(liste)) if i1<i2]
    return l

if __name__=='__main__':
    import doctest
    doctest.testmod()

