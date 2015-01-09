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
![Overview of remains of the Roman amphitheater in Rossfeld, Bern](https://github.com/DRONARCHers/DRONARCH/blob/master/results/09_01_15/overview.png)
![Top view of remains of the Roman amphitheater in Rossfeld, Bern](https://github.com/DRONARCHers/DRONARCH/blob/master/results/09_01_15/top.png)
![Center view of remains of the Roman amphitheater in Rossfeld, Bern](https://github.com/DRONARCHers/DRONARCH/blob/master/results/09_01_15/inside.png)
![Detail view of remains of the Roman amphitheater in Rossfeld, Bern](https://github.com/DRONARCHers/DRONARCH/blob/master/results/09_01_15/detail.png)