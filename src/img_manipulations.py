import cv2, helpers
from debug import debug

__author__ = 'niclas'


def check_and_resize_all(src_dir, dest_dir, size, formats):
    """
    Scans the source directory and saves a scaled copy of each image, that is too large in the destination directory

    :param src_dir: Directory to scan
    :param dest_dir: Directory to save copies in
    :param size: The maximum allowed size
    :param formats: List containing the image formats to look for
    :return:
    """
    resized_files = []
    orig_files = []

    for file in helpers.get_files_with_ending(src_dir, formats): # all imgs in src_dir
        orig_files.append(file)
        resized_files.append(check_and_resize(file, dest_dir, size))
    return (resized_files,orig_files)


def check_and_resize(file, dest_dir, (height, width)):
    """
    Checks whether an image is too large. If it is, a smaller copy is stored in the specified folder
    :param file: The image
    :param dest_dir: Directory to save the scaled image in
    :param (height, width): Maximum dimension
    :return:
    """
    img = cv2.imread(file)
    (img_h, img_w, img_d) = img.shape

    if img_w>width or img_h > height: #check whether image is too large
        img = resize(img, (height, width))
        (img_h_sm, img_w_sm, img_d_sm) = img.shape

        # create new file name
        name = file.split('/')
        name = ''.join([dest_dir,name[-1]])

        #CAUTION: bundler only accepts files ending with exactly .jpg or .bmp. .JPG or jpeg is not allowed
        name_decomp = name.split('.')
        if name_decomp[-1] in ['JPG','JPEG','jpeg', 'jpg']:
            name_decomp[-1] = 'jpg'
        elif name_decomp[-1] in ['BMP', 'bmp']:
            name_decomp[-1] = 'bmp'
        else:
            debug(2, 'File ',name, 'has invalid format.')

        name = '.'.join(name_decomp)
        success =  cv2.imwrite(name, img)

        if success:
            debug(0, 'Image is to large. Resized ',file, ' from ', img_h, ',',img_w, ' to ', img_h_sm, ',', img_w_sm,' and saved as ', name)
        else:
            debug(2, 'Could not scale or store image ', file, ' to ', name)
    else: #is no resize is needed, just copy the file
        name = dest_dir + helpers.get_filename_from_path(file)
        success =  cv2.imwrite(name, img)

        if success:
            debug(0, 'Image ', file, ' has been copied to', dest_dir)
        else:
            debug(2, 'Could not scale or store image ', file, ' to ', name)

    return name

def resize(img, (height, width)):
    """
    Resizes an image to be no larger than the specified dimension but as large as possible
    :param img: Image to be resized
    :param (height, width): Maximum dimension
    :return:
    """
    (img_h, img_w, img_d) = img.shape
    factor = min(float(width)/img_w, float(height)/img_h)
    img = cv2.resize(img, (0,0), fx=factor, fy=factor)
    return img

def get_size(image):
    img = cv2.imread(image)
    (img_h, img_w, img_d) = img.shape
    return (img_h, img_w, img_d)

# check_and_resize('../imgs/IMG_0632.JPG','../temp_imgs/', (1000,1000))
# check_and_resize_all(src_dir=Dronarch.orig_img_dir, dest_dir=Dronarch.temp_img_dir, size=(1000,1000), formats=Dronarch.img_formats)
# print get_size('../imgs/IMG_0621.JPG')