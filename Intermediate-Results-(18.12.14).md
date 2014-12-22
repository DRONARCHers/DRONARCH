A lot of work  has been done so far and a lot more will be done the next few weeks. Here's a short overview

### So far and problems
The main pipeline is now (18.12.14) implemented but results are still very bad. I think I will have to tweak the usage of the intrinsic camera parameters in bundler.
The dense point cloud is basically still just noise. Here my guess is, that the camera position estimated in the bundle adjustment is not good enough. I will try to fix that.
In the meanwhile I'm not sure if the video I've used for testing was good enough.

### What's next
So I will focus on the drone control to get some good images myself and then tweak the capture and reconstruction process together.

### Results
The first results using 50 images with 5 imgs/sec looks like that:
![Side view of bundler output using 50 images and 5 imgs/sec](https://github.com/DRONARCHers/DRONARCH/blob/master/results/18_12_14/int_res_side.png)
![Top view of bundler output using 50 images and 5 imgs/sec](https://github.com/DRONARCHers/DRONARCH/blob/master/results/int_res_top.png)

The input is the following video [youtube: Parrot AR.Drone 2.0: Director Mode camera demo (HD)](https://www.youtube.com/watch?v=YGk2ghqoHug)

The red, green and yellow dots are the camera positions and the other points are reconstructed scene points.
![Top view of bundler. The green area is the "front pillars" of the building, the red the actual walls](https://github.com/DRONARCHers/DRONARCH/blob/master/results/18_12_14/int_res_top_mark.png)
In this image I marked the "front pillars" of the building green, and with read the actual walls.
Note that the walls behind have looots of outliers.
They are marked red in the following image.

![Side view of bundler. Red area is outliers](https://github.com/DRONARCHers/DRONARCH/blob/master/results18_12_14//int_res_side_mark.png)



