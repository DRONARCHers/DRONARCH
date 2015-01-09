### Roman amphitheater in Rossfeld, Engehalbinsel Bern
This week I took a set of 124 images and a video of the Roman amphitheater in the Rossfeld on the Engehalbinsel near Bern. The Roman vicus (city) Brenodurum is known on this peninsula, and before that a celtic settlement. Today the bath and the amphitheater is still visible, to be precise it is mostly a modern reconstruction. Still I regarded it is a good object for my tests.

### Long processing time
I worked with a images resolution of 3000x2000px, the larges I've ever tried so far. The processing took ~27h. The substeps were:
* Detect SIFT features (1h)
* Match SIFT features (18h!)
* Bundle Adjustment (2h)
* Multi-view stereo (6h)

### Results
The following are the resulting dense point clouds.
The detail level varies quite a bit. Also the multi-view stereo part is done in several patches for memory reasons. These patches overlap sometimes with different point density. So the quality can be enhanced with some manual post-processing.
Note that there are still many holes in the model. I'm trying to tweak the pipeline such that these get smaller. I also tried to make a mesh in MeshLab again, but it keeps crashing. @simplay: I'll get back to you the next days :-)
![Overview of remains of the Roman amphitheater in Rossfeld, Bern](https://github.com/DRONARCHers/DRONARCH/blob/master/results/09_01_15/overview.png)
![Top view of remains of the Roman amphitheater in Rossfeld, Bern](https://github.com/DRONARCHers/DRONARCH/blob/master/results/09_01_15/top.png)
![Center view of remains of the Roman amphitheater in Rossfeld, Bern](https://github.com/DRONARCHers/DRONARCH/blob/master/results/09_01_15/inside.png)
![Detail view of remains of the Roman amphitheater in Rossfeld, Bern](https://github.com/DRONARCHers/DRONARCH/blob/master/results/09_01_15/detail.png)