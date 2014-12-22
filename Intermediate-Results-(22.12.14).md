### Using intrinsics K
I have worked on better usage of the intrinsic parameters (K-matrix) acquired doing a chessboard calibration.
This simplifies the problem of finding the essential matrix E a lot.
The fundamental matrix F is estimated using the epipolar constraints. Since E = K*F*K^T, E is easy to find if K is known.

### Results
This is the sparse point cloud produced by bundler. You see, that the green areas show some nice details and the red area has fewer outliers than previously.
![Top view of bundler output using 30 images and 10 imgs/sec](https://github.com/DRONARCHers/DRONARCH/blob/master/results/22_12_14/side_mark.png)

Using different images I could produce a dense point cloud that captures some pretty details (green area). But there are still this weird artifacts (red). I think, these are wrong matches of the brown pillars. There are several pillars, that look very similar and can easily be detected as the same.

![Top view of PMVS output](https://github.com/DRONARCHers/DRONARCH/blob/master/results/22_12_14/side_2_mark.png)