"""
Provides functions for image manipulation such as resizing
"""

__author__ = 'niclas'

import cv2,os
from PIL import Image, ExifTags
from dronarch.helpers.debug import debug
from dronarch.helpers import helpers
from ccd_width import CCD_WIDTHS



class DronarchImage:
    def __init__(self, path, focal_length=None, size=None):
        self.path = path
        self.focal_length = focal_length
        self.size = size

    def get_size(self):
        """
        Returns the size in pixel of an image
        :param image:
        :return: (height, width, color depth)
        """
        if self.size == None:
            img = self.read_image()
            self.size = img.shape
        return self.size

    def get_width(self):
        return self.get_size()[0]

    def get_height(self):
        return self.get_size()[1]

    def read_image(self):
        """
        Reads this image and returns it as a opencv image
        """
        file_name = self.path.express()
        if not os.path.isfile(file_name):
            debug(2, 'Cannot read image ', file_name)
            return
        img = cv2.imread(file_name)
        return img

    def save(self, img):
        success = cv2.imwrite(self.path.express(), img)
        if not success:
            debug(1, 'Could not save image to ', self.path.express())
        return success

    def get_focal_length(self):
        """Extracts (pixel) focal length from image where available.
        """
        if self.focal_length == None:
            image_name = self.path.express()
            debug(0, 'Extracting EXIF tags from image {0}'.format(image_name))

            tags = {}
            with open(image_name, 'rb') as fp:
                img = Image.open(fp)
                if hasattr(img, '_getexif'):
                    exifinfo = img._getexif()
                    if exifinfo is not None:
                        for tag, value in exifinfo.items():
                            tags[ExifTags.TAGS.get(tag, tag)] = value

                # Extract Focal Length
                focalN, focalD = tags.get('FocalLength', (0, 1))
                focal_length = float(focalN)/float(focalD)

                # Extract CCD Width (Prefer Lookup Table)
                make_model = tags.get('Make', '') + ' ' + tags.get('Model', '')
                if CCD_WIDTHS.has_key(make_model.strip()):
                    ccd_width = CCD_WIDTHS[make_model.strip()]
                else:
                    fplaneN, fplaneD = tags.get('FocalPlaneXResolution', (0, 1))
                    if fplaneN != 0:
                        ccd_width = 25.4*float(self.get_width())*float(fplaneD)/float(fplaneN)
                        debug(0,'Using CCD width from EXIF tags')
                    else:
                        ccd_width = 0

                if ccd_width == 0:
                    debug(1,'No CCD width available for camera {0}]'.format(
                        make_model))

                if (self.get_width()==0 or focalN==0 or ccd_width==0):
                    debug(1,'Could not determine pixel focal length')


                # Compute Focal Length in Pixels
                scales_focal_length = self.get_width() * (focal_length / ccd_width)
                debug(0,'Focal length (pixels) = {0}'.format(scales_focal_length))

                self.focal_length = scales_focal_length
        return self.focal_length

    def resize_and_save_new(self, (height, width), new_path):
        """
        Resizes an image to be no larger than the specified dimension but as large as possible
        :param img: Image to be resized
        :param (height, width): Maximum dimension
        :param new_path: the Path of the new Image object
        :return: success, factor: boolean that says whether saving was successful. Scaling factor
        """
        img = self.read_image()
        (img_h, img_w, img_d) = img.shape
        factor = min(float(width)/img_w, float(height)/img_h)
        img = cv2.resize(img, (0,0), fx=factor, fy=factor)
        size = img.shape
        new_focal_length = self.get_focal_length()*factor
        new_image = DronarchImage(path=new_path, focal_length=new_focal_length, size=size)
        success =new_image.save(img)

        if success:
            debug(0, 'Image is too large. Resized ',self.path.express(), ' from ', img_h, ',',img_w, ' to ', size[0], ',', size[1],' The scale factor is '+str(factor),'  Saved as ', new_path.express())
        else:
            debug(2, 'Could not scale or store image ', self.path.express(), ' to ', new_path.express())

        return new_image, factor

    def __str__(self):
        return str(self.path)

def check_and_resize_all(src_dir, dest_dir, max_size, formats, use_images_with_same_size_only=False):
    """
    Scans the source directory and saves a scaled copy of each image, that is too large in the destination directory

    :param src_dir: Directory to scan
    :param dest_dir: Directory to save copies in
    :param size: The maximum allowed size
    :param formats: List containing the image formats to look for
    :return:
    """
    resized_images = []
    orig_images = []
    scale = -1
    size = (-1,-1)
    for file in helpers.get_files_with_ending(src_dir, formats): # all imgs in src_dir
        image = DronarchImage(file)
        orig_images.append(image)
        new_image, factor = check_and_resize(image, dest_dir, max_size)

        #Use first scale factor as initial value
        if scale==-1:
            scale = factor
            size = new_image.get_size()

        # If images needn't be the same size, just add them
        if not use_images_with_same_size_only:
            resized_images.append(new_image)
        else: #only add images of the size as the first image
            #Check whether the image is the same size as the first iamge
            if factor==scale:
                resized_images.append(new_image)
            else:
                debug(1, 'Image ', new_image.path.express(), ' has been rejected. The size does not fit the other images')

    return resized_images, scale, size


def check_and_resize(image, dest_dir, (height, width)):
    """
    Checks whether an image is too large. If it is, a smaller copy is stored in the specified folder
    :param image: The image
    :param dest_dir: Directory to save the scaled image in
    :param (height, width): Maximum dimension
    :return:
    """
    file_name = image.path.express()
    (img_h, img_w, img_d) = image.get_size()


    #CAUTION: bundler only accepts files ending with exactly .jpg or .bmp. .JPG or jpeg is not allowed
    name_decomp = image.path.path_list[-1].split('.')
    if name_decomp[-1] in ['JPG','JPEG','jpeg', 'jpg']:
        name_decomp[-1] = 'jpg'
    elif name_decomp[-1] in ['BMP', 'bmp']:
        name_decomp[-1] = 'bmp'
    else:
        debug(2, 'File ',file_name, 'has invalid format.')

    # create new file name
    name_ending = '.'.join(name_decomp)
    new_path = dest_dir.new_app_path(name_ending)
    new_name = new_path.express()

    if img_w>width or img_h > height: #check whether image is too large
        new_image, factor = image.resize_and_save_new((height, width), new_path=new_path)

        (img_h_sm, img_w_sm, img_d_sm) = new_image.get_size()


    else: #is no resize is needed, just copy the file
        new_image = Image(path=new_path, size=image.get_size(), focal_length= image.get_focal_length())
        success = new_image.save()
        #If the size has not been changed, us old values
        factor = 1
        img_w_sm = img_w
        img_h_sm = img_h

        if success:
            debug(0, 'Image ', file_name, ' has been copied to', dest_dir.express())

    return new_image, factor




# check_and_resize('../imgs/IMG_0632.JPG','../temp_imgs/', (1000,1000))
# check_and_resize_all(src_dir=Dronarch.orig_img_dir, dest_dir=Dronarch.temp_img_dir, size=(1000,1000), formats=Dronarch.img_formats)
# print get_size('../imgs/IMG_0621.JPG')