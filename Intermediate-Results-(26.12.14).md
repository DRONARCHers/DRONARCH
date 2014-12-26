### PMVS did well
For the first time I was able to produce a dense point cloud using PMVS. It's been implemented since a few weeks, but I never got it working until now.
The problem was (and still is) that I either don't get correct (or maybe just correctly scaled) intrinsics doing the chessboard calibration. Or bundlers option --intrinsics has some problems. I will dig into that the next days.

### Results
This is a test scene I've recorded in my office. I used 50 images and 3 images per second. I still don't know what configurations work best.
![Office scene, right view](https://github.com/DRONARCHers/DRONARCH/blob/master/results/26_12_14/right_view_mark.png)
Note the details of the bottles (marked green. No, I don't want to hear any questions on why these bottles were there^^).

![Office sceen, left view](https://github.com/DRONARCHers/DRONARCH/blob/master/results/26_12_14/left_view_mark.png)
The screen made some problems. It's position is partly incorrect. Also note that the metal of the table's legs are in correct position, even though they are glossy.

![Office scene, camera positions](https://github.com/DRONARCHers/DRONARCH/blob/master/results/26_12_14/camera_pos_mark.png)
In the green area, the cameras are marked as red/yellow and green/yellow points. The movement is still very small and structured. I wonder how well this will work with less images and less structure.
The wall behind the desk (marked red) caused some troubles. There are no unique features and I assume, there are some features mismatched. I hope to fix this by adjusting some parameters of the matching.

### Conclusion
This is a very nice result so far. The image quality is not very good and taken without any planing. So improving the quality of the video and fixing the issue with the intrinsic parameters might give some much better results. Because this is still not good enough.
It also turned out that choosing a good initial pair of images is key. I will have to come up with a more convenient way for this. So far they are hardcoded.