import cv2,os,ntpath,helpers
from debug import debug


__author__ = 'niclas'

def check_and_extract_all_videos(src_dir, dest_dir,formats, imgs_per_sec, start_frame, no_images):
    videos = helpers.get_files_with_ending(src_dir, formats)
    names = []
    if len(videos)>0:
        for video in videos:
            name = video2image(video, dest_dir, imgs_per_sec, start_frame, no_images)
            names.extend(name)
    return names

def video2image(video, dest_folder, imgs_per_sec, start_frame=0, no_images=50):
    """
    Extracts images from a video and stores them into the defined folder.
    the first image will be at start_frame. From there, depending on imgs_per_seconds, further images will be extracted until no_images many have been created.

    :param video: Path to the video file
    :param dest_folder: The images will be stored under this path
    :param star_frame: The frame to start extraction at
    :param imgs_per_sec: How many images should be extracted per second
    :param no_images: How many images should be extracted.
    :return:
    """
    #test if video exists
    if not os.path.isfile(video):
        debug(1, 'No valid file ', video)
        return

    #get file name
    file_name,ending = ntpath.basename(video).split('.')

    #open video
    cap = cv2.VideoCapture(video)

    fps = int(cap.get(cv2.cv.CV_CAP_PROP_FPS))
    duration = int(cap.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT))

    step = int(fps/imgs_per_sec)
    end = min(duration, start_frame+step*no_images)
    no_img_proc = 0

    names = []
    for t in range(start_frame,end,step):
        cap.set(cv2.cv.CV_CAP_PROP_POS_FRAMES,t)
        ret, frame = cap.read()
        name = dest_folder+file_name+str(no_img_proc)+'.jpg'
        names.append(name)
        cv2.imwrite(name, frame)

        no_img_proc = no_img_proc+1

    debug(0, no_img_proc, ' images have been written to ', dest_folder)
    return names