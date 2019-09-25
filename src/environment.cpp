
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

/*This functions does everything that cityBlock function does
 * except that it uses PCL inbuild functions for Segmentation ,
 * clustering and filtering is not performed. This function is
 * not used in the project
 * */
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    Color raycolor = Color(255,0,0);
    // TODO:: Create lidar sensor 
    Lidar *Lidar_Obj= new Lidar(cars,0);

    /* Read a scene*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud = Lidar_Obj->scan();
    //renderRays(viewer,Lidar_Obj->position,pointcloud);
    //renderPointCloud(viewer,pointcloud,"Example",raycolor);
    // TODO:: Create point processor

    /*Segment the scene into plane and objects*/
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(pointcloud, 100, 0.2);
    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane_RANSAC(pointcloud, 100, 0.2);
    //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    //renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    /*Identify the clusters in the objects that match the thresholds*/
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    /*Append bounding boxes around identified clusters and color them*/
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
          std::cout << "cluster size ";
          pointProcessor.numPoints(cluster);
          renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

          Box box = pointProcessor.BoundingBox(cluster);
          renderBox(viewer,box,clusterId);

          ++clusterId;
    }
}

/* cityBlock function is the main function that does operations to identify objects in the scene,
 * following operations are performed:
 * Filter: Crop the scene to requested dimensions and remove the points from the roof top
 * Segmentation: Segment the scene to create to clouds one for road and another for objects.
 * Clustering: Identify the clusters from objects
 * Bounding Box: Append a bounding box for each of the clustered object
 * */
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;
	/*Filter: Crop the scene to requested dimensions and remove the points from the roof top*/
	filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.15 , Eigen::Vector4f (-20, -6, -2, 1), Eigen::Vector4f ( 30, 7, 5, 1));
	/*Segmentation: Segment the scene to create to clouds one for road and another for objects.*/
	std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane_RANSAC(filterCloud, 50, 0.2);
	/*Render the road to the viewer*/
	renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
	/*Clustering: Identify the clusters from objects*/
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering_euclideanCluster(segmentCloud.first, 0.3, 10, 1000);

	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
	/*Render the objects along with bounding boxes to the viewer*/
	for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
	{
	   std::cout << "cluster size ";
	   pointProcessorI->numPoints(cluster);
	   renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);

	   Box box = pointProcessorI->BoundingBox(cluster);
	   renderBox(viewer,box,clusterId);

	   ++clusterId;
	}

}
//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    while (!viewer->wasStopped ())
    {
      // Clear viewer
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      // Load pcd and run obstacle detection process
      inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
      //renderPointCloud(viewer,inputCloudI,"Example");

      /*Call cityBlock to identify the objects*/
      cityBlock(viewer, pointProcessorI, inputCloudI);

      // Increment to next scene and if the last scene is reached loop back to first scene.
      streamIterator++;
      if(streamIterator == stream.end())
        streamIterator = stream.begin();
      viewer->spinOnce ();
    }

}
