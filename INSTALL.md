# Install

Instructions for installing DRONARCH

## On OSX
For OSX users

### Create dir
In Terminal:

```mkdir dronarch```

```cd dronarch```

### Get DRONARCH source
In Terminal:

```git clone https://github.com/DRONARCHers/DRONARCH.git -b OSX .```

### python 2.7.10
Python 2.7.10 64bit
From https://www.python.org/downloads
Install it

### numpy 1.9.2
- Using pip: `pip install numpy`
OR:
- Download from http://www.scipy.org/scipylib/download.html
- Install according to instructions
- You may try: `python setup.py install --user   # installs to your home directory -- requires Python >= 2.6`
- If not done during installation, add bin-path of numpy to PYTHONPATH variable

### opencv 2.4.9
- Install via *homebrew* (see: https://jjyap.wordpress.com/2014/05/24/installing-opencv-2-4-9-on-mac-osx-with-python-support/)
- `brew tap homebrew/science`
- `brew install opencv` if this fails try `brew update` first.
OR:
- Download from http://opencv.org/downloads.html
- Add [PATH TO OPENCV]/opencv/build/python/2.7/x86 to PYTHONPATH variable

- Add `/usr/local/Cellar/opencv/2.4.12/lib/python2.7/site-packages` to your PYTHONPATH
- `echo 'PYTHONPATH=$PYTHONPATH:/usr/local/Cellar/opencv/2.4.12/lib/python2.7/site-
packages'>>$HOME/.bash_profile`
- `echo 'export PYTHONPATH'>>$HOME/.bash_profile`
- Try in the python shell `python -c 'import cv2'` to see whether the installation was successful.

### bundler

Some preparing steps are needed to install bundler

#### jhead
- Install *jhead* using *homebrew*: `brew install jhead`

#### ImageMagick
- Download *ImageMagik* using *homebrew* `brew install ImageMagick` or the OSX binaries from www.imagemagick.org/script/binary-releases.php

#### Pillow
- Download *Python Imaging*  binaries *Pillow-2.9.0-cp27-none-macosx_10_6_intel.macosx_10_9_intel.macosx_10_9_x86_64.macosx_10_10_intel.macosx_10_10_x86_64.whl* (or any 2.*) from https://pypi.python.org/pypi/Pillow/2.9.0
- Install Pillow using `pip install <your pillow .whl file>`
- Try `from PIL import Pillow` to make sure the installation is complete.

#### PMVS/CMVS
- Get the binaries form https://github.com/pmoulon/CMVS-PMVS `git clone git@github.com:pmoulon/CMVS-PMVS.git`to some place you like

#### sift
- Download *sift* binaries http://www.cs.ubc.ca/~lowe/keypoints/

#### bundler
- Download *bundler* binaries from https://github.com/TheFrenchLeaf/Bundler.git
- Or `git clone git@github.com:TheFrenchLeaf/Bundler.git`

### Configuring the pathes

Now you're ready for a hot config session.
All your pathes MUST be termineted by /.
I'm not sure whether ~/ pathes are working.
- In the dronarch base directory open the file config/dronarch.cfg
- *base_dir*: Your projects root (e.g. `/Users/<name>/dev/python/dronarch/` )
- *bundler_bin_dir*: The directory containing your bundlers binaries (e.g. `/Users/<name>/dev/c/Bundler/bin/`)
- *cmvs_bin_path*: The directory containing the CMVS binaries (e.g. `/Users/<name>/dev/c/CMVS-PMVS/binariesWin-Linux/mac_osx_fat/`)
- *pmvs_bin_path*: The directory containing the CMVS binaries (e.g. `/Users/<name>/dev/c/CMVS-PMVS/binariesWin-Linux/mac_osx_fat/`)
- *vid_calib_img_dir* :Directorie containing calibration images for the video camera (currently not really used. e.g. `/Users/<name>/dev/python/dronarch/vid_calib/`)
- *img_calib_img_dir* :Directorie containing calibration images for the video camera (currently not really used. e.g. `/Users/<name>/dev/python/dronarch/img_calib/`)
- *calib_file_path*: Thats where calibration files are stored. They should be created when running DRONARCH using a calibration. (e.g. `/Users/<name>/dev/python/dronarch/config/calib_file.cfg`)

