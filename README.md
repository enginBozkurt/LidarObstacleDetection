# Lidar Obstacle Detection

```diff
 The main goal of the project is to filter, segment, and cluster  
 real point cloud data to detect obstacles in a driving environment.
```
![ezgif com-video-to-gif](https://user-images.githubusercontent.com/30608533/65621434-65750100-dfcc-11e9-864f-b9a93de1b16e.gif)

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Also we can tell a little bit about the object that was hit by measuring the intesity of the returned signal. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. While lidar sensors gives us very high accurate models for the world around us in 3D, they are currently very expensive, upwards of $60,000 for a standard unit.

**Radar data** is typically very sparse and in a limited range, however it can directly tell us how fast an object is moving in a certain direction. This ability makes radars a very pratical sensor for doing things like cruise control where its important to know how fast the car infront of you is traveling. Radar sensors are also very affordable and common now of days in newer cars.

**Sensor Fusion** by combing lidar's high resoultion imaging with radar's ability to measure velocity of objects we can get a better understanding of the sorrounding environment than we could using one of the sensors alone.

```diff
 This project implements pipeline for converting the raw LIDAR sensor measurements into trackable objects. 
 It implements filtering, segmentation, clustering, boundbox routines. Filtering was performed using 
 the PCL functions. 
 Functions were implemented for segmentation and clustering. The pipeline details are as the following.
```


## PipeLine 
Following are the steps for the pipeline
- Read a PCD file.
- Filter the raw LIDAR data 
- Segment the filtered data to identify road and objects
- Clustering: from objects identify the clusters.
- For each cluster append a bounding box. 
- Rendering: Render the objects and road to the viewer. 



Following are the details for a few of the above steps. 

## Filtering
The raw LIDAR data is filtered, this helps in increasing the processing speed and reduce false targets. The following list of operations was performed. 
 - Downsampling: points are converted to voxels to a given dimension. PCL function VoxelGrid was used for this operation.
 - Crop: Remove all the points that are outside the limits. PCL function CropBox was used for this operation. Limits were found by visually inspecting the scene. 
 - RoofCrop: Remove roof points, dimensions of roof are identified by visually inspecting the scene.PCL function CropBox was used for this operation.
 
## Segmentation
The filtered output is segmented. Segmentation divides the scene into plane and objects. Plane refers to the road surface. This step helps in differentiating  the drivable area from obstacles. RANSAC algorithm was used. Following steps were performed to rrealize this function.
  - Randomly choose 3 points from the cloud, fit a plane using these points.
  - Loop through all the points in the cloud, for each of the point calculate
     - The distance to the plane created above. If the distance is below distanceTol then add the index to a temporary set
     - Store the temporary vector if the size is more than previously identified indices.
  - Repeat the above steps for maxIterations : 
  
 maxIterations value was obtained by doing a couple of trails with different values, the value that gives the best result was choosen. 
 
 ## Clustering
 Clustering helps in identifying different objects in the scene. Euclidean Clustering mechanism was implemented. Euclidean Clustering uses KDTree , all the points in the cloud are used to form a 3-Dimensional KDTree. After forming KDTree a point is taken from the cloud and all the points that are within a distance to this point are identified, each of the identified points is used and its adjacent points that are within a distance are identified this process is repeated until there are no points that satisfy the distance criteria are left,all the identified points form a cluster, after that a new point that is not processed is picked and the process is left. All the clusters with the number of points in the range of (minSize, maxSize) are retained and the rest are discarded. Each of these clusters forms a distinct object. 

## Bounding Box 
 For each of the cluster, a bounding box is fitted. To fit a box min and max coordinates of a cluster are used. Using renderbox function boxes are drawn. 
 


## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/enginBozkurt/LidarObstacleDetection.git
$> cd LidarObstacleDetection
$> mkdir build && cd build
$> cmake ..
$> make
$> ./environment
```

### Windows 

http://www.pointclouds.org/downloads/windows.html

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew 
	```bash
	$> brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
	```bash
	$> brew tap brewsci/science
	```
4. view pcl install options
	```bash
	$> brew options pcl
	```
5. install PCL 
	```bash
	$> brew install pcl
	```


## Notes about the mentioned Algorithms and Point Cloud Library (PCL) functions

![Screenshot_1](https://user-images.githubusercontent.com/30608533/65598405-013e4700-dfa4-11e9-9ee4-d48650cd486d.jpg)

![Screenshot_2](https://user-images.githubusercontent.com/30608533/65598416-0602fb00-dfa4-11e9-967b-19cb9db49b09.jpg)



***class pcl::CropBox< PointT > CropBox is a filter that allows the user to filter all the data inside of a given box.***

![Screenshot_3](https://user-images.githubusercontent.com/30608533/65599785-44e68000-dfa7-11e9-9974-15b276202b76.jpg)



***Euclidean Cluster Extraction Tutorial***

Source: http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction

### Point Cloud Library (PCL) functions installation guide and notes

## For Linux Ubuntu 16.04

- https://larrylisky.com/2014/03/03/installing-pcl-on-ubuntu/

- libvtk needed to be updated to libvtk6-dev instead of (libvtk5-dev). The linker was having trouble locating libvtk5-dev while building, but this might not be a problem for everyone.

- BUILD_visualization needed to be manually turned on, this link shows you how to do that, http://www.pointclouds.org/documentation/tutorials/building_pcl.php

## For Linux Ubuntu 18.04

```bash
### Clone latest PCL(Current is 1.9.1)
sudo apt-get update
sudo apt-get install -y git

cd ~/Documents
git clone https://github.com/PointCloudLibrary/pcl.git pcl-trunk
ln -s pcl-trunk pcl
cd pcl

### Install prerequisites
sudo apt-get install -y g++
sudo apt-get install -y cmake cmake-gui
sudo apt-get install -y doxygen
sudo apt-get install -y mpi-default-dev openmpi-bin openmpi-common
sudo apt-get install -y libflann1.9 libflann-dev
sudo apt-get install -y libeigen3-dev
sudo apt-get install -y libboost-all-dev
sudo apt-get install -y libvtk6-dev libvtk6.3 libvtk6.3-qt
sudo apt-get install -y 'libqhull*'
sudo apt-get install -y libusb-dev
sudo apt-get install -y libgtest-dev
sudo apt-get install -y git-core freeglut3-dev pkg-config
sudo apt-get install -y build-essential libxmu-dev libxi-dev
sudo apt-get install -y libusb-1.0-0-dev graphviz mono-complete
sudo apt-get install -y qt-sdk openjdk-11-jdk openjdk-11-jre
sudo apt-get install -y phonon-backend-gstreamer
sudo apt-get install -y phonon-backend-vlc
sudo apt-get install -y libopenni-dev libopenni2-dev

### Compile and install PCL
mkdir release
cd release
cmake -DCMAKE_BUILD_TYPE=None -DBUILD_GPU=ON -DBUILD_apps=ON -DBUILD_examples=ON ..
make
sudo make install

```

## Final Notes
This project is inspired by Udacity Sensor Fusion Engineer Nanodegree course.


