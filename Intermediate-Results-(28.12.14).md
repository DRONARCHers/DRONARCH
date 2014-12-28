### Parameter tweaking
Using 250 images (2-3h processing time) resulted in this nice point cloud
![PMVS output 250 images](https://github.com/DRONARCHers/DRONARCH/blob/master/results/28_12_14/left.png)
![PMVS output 250 images](https://github.com/DRONARCHers/DRONARCH/blob/master/results/28_12_14/right.png)
The detail level is really nice. The paper is not readable, but you can see the different paragraphs.
![PMVS output 250 images](https://github.com/DRONARCHers/DRONARCH/blob/master/results/28_12_14/paper_mark.png)
There are still some large holes in the cloud. These are mainly the uniform regions. I hope to be able to improve this by tweaking the parameters.
![PMVS output 250 images](https://github.com/DRONARCHers/DRONARCH/blob/master/results/28_12_14/screen_mark.png)

### Mesh and textures
Today I tried to create a mesh out of this point cloud and texture it. Bundler's output files can be imported into Meshlab and then the camera positions and images can be used for texturing. Unfortunately Meshlab crashes CONSTANTLY... I can not import the bundle file into Meshlab using Ubuntu, but on Windows this works... but the poisson reconstruction makes it crash,which is then working on Ubuntu. So my work-flow includes constantly changing the OS I'm using. To good there are enough computers here in the lab :-)
