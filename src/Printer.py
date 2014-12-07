import sys

__author__ = 'niclas'

class prinzPrinter:
    """
    Overwrites the default stdout
    """
    def __init__(self):
        self.old_stdout=sys.stdout

    def write(self, text):
        text = text.rstrip()
        if len(text) == 0: return
        self.old_stdout.write('custom Print--->' + text + '\n')

# myPrint = Printer()
# sys.stdout = myPrint
# print('test')