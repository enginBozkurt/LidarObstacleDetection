# Lidar Obstacle Detection

```diff
 The main goal of the project is to filter, segment, and cluster  
 real point cloud data to detect obstacles in a driving environment.
```
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
- Rendering:Render the objects and road to the viewer. 



Following are the details for a few of the above steps. 

## Filtering
The raw LIDAR data is filtered , this helps in increasing the processing speed and reduce false targets. Following list of operations were performed. 
 - Downsampling: points are converted to voxels to a given dimension. PCL function VoxelGrid was used for this operation.
 - Crop: Remove all the points that are outside the limits. PCL function CropBox was used for this operation. Limits were found by visually inspecting the scene. 
 - RoofCrop: Remove roof points , dimensions of roof are identified by visually inspecting the scene.PCL function CropBox was used for this operation.
 
## Segmentation
Filtered output is segmented. Segmentation divides the scene into plane and objects. Plane refers the road surface. This step helps in differetiating the drivable area from obstacles. RANSAC algorithm was used. Following steps were performed to realise thie function.
  - Randomly choose 3 points from the cloud, fit a plane using these points.
  - Loop through all the points in the cloud, for each of the point calculate
     - The distance to the plane created above. If the distance is below distanceTol then add the index to a temporary set
     - Store the temporary vector if the size is more than previously identified indices.
  - Repeat above steps for maxIterations : 
  
 maxIterations value was obtained by doing a couple of trails with different values, value that gives best result was choosen. 
 
 ## Clustering
 Clustering helps in identifying different objects in the scene. Euclidean Clustering mechanism was implemented. Euclidean Clustering uses KDTree , all the points in the cloud are used to form a 3-Dimensional KDTree. After forming KDTree a point is taken from the cloud and all the points that are within a distance to this point are identified, each of the identified points is used and its adjacent points that are with in a distance are identified this process is repeated untill there are no points that satisfy the distance criteria are left,all the identified points form a cluster, after that a new point that is not processed is picked and the process is left. All the clusters with number of points with in (minSize, maxSize) are retained rest are discarded. Each of these clusters form a distinct object. 

## Bounding Box 
 For each of the cluster a bounding box is fitted. To fit a box min and max coordinates of a cluster are used. Using renderbox function boxes are drawn. 
 
## Output
 Screen shots of output in different views are placed in Output folder.


## Installation

### Ubuntu 

```bash
$> sudo apt install libpcl-dev
$> cd ~
$> git clone https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
$> cd SFND_Lidar_Obstacle_Detection
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





