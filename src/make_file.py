__author__ = 'niclas'

file = 'imgs.txt'
dicti = {'./sm-9.jpg': 3946.869070208729, './sm-11.jpg': 3946.869070208729, './sm-13.jpg': 3946.869070208729, './sm-10.jpg': 3946.869070208729, './sm-3.jpg': 3946.869070208729, './sm-1.jpg': 3946.869070208729, './sm-7.jpg': 3946.869070208729, './sm-5.jpg': 3946.869070208729, './sm-6.jpg': 3946.869070208729, './sm-4.jpg': 3946.869070208729, './sm-14.jpg': 3946.869070208729, './sm-8.jpg': 3946.869070208729, './sm-12.jpg': 3946.869070208729, './sm-2.jpg': 3946.869070208729, './sm-0.jpg': 3946.869070208729}
with open(file, 'w') as fp:
            for image,value in dicti.items():
                if value == None: fp.write(image + '\n')
                else: fp.write(' '.join([image, '0', str(value), '\n']))
            image_list_file = fp.name
print(image_list_file)