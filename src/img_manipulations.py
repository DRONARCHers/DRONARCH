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

    for file in helpers.get_files_with_ending(src_dir, formats): # all imgs in src_dir
        resized_files.append(check_and_resize(file, dest_dir, size))
    return resized_files


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
        print name
        name = ''.join([dest_dir,name[-1]])

        success =  cv2.imwrite(name, img)
        if success:
            debug(0, 'Image is to large. Resized ',file, ' from ', img_h, ',',img_w, ' to ', img_h_sm, ',', img_w_sm,' and saved as ', name)
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

# check_and_resize('../imgs/IMG_0632.JPG','../temp_imgs/', (1000,1000))
# check_and_resize_all(src_dir=Dronarch.orig_img_dir, dest_dir=Dronarch.temp_img_dir, size=(1000,1000), formats=Dronarch.img_formats)