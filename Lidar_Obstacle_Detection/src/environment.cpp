/* \author Aaron Brown */
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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar * lidar = new Lidar(cars, 0); // Instantiate on the heap
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();

    // Test points for PCA bounding boxes
    inputCloud->points.push_back(pcl::PointXYZ(0.00, -5.00, 1.0));
    inputCloud->points.push_back(pcl::PointXYZ(0.00, -5.25, 1.0));
    inputCloud->points.push_back(pcl::PointXYZ(0.25, -5.25, 1.1));
    inputCloud->points.push_back(pcl::PointXYZ(0.25, -5.50, 1.1));
    inputCloud->points.push_back(pcl::PointXYZ(0.50, -5.50, 1.2));
    inputCloud->points.push_back(pcl::PointXYZ(0.50, -5.75, 1.2));
    inputCloud->points.push_back(pcl::PointXYZ(0.75, -5.75, 1.3));
    inputCloud->points.push_back(pcl::PointXYZ(0.75, -6.00, 1.3));
    inputCloud->points.push_back(pcl::PointXYZ(1.00, -6.00, 1.4));
    inputCloud->points.push_back(pcl::PointXYZ(1.00, -6.25, 1.4));

    inputCloud->points.push_back(pcl::PointXYZ(0.00, -5.00, 0.5));
    inputCloud->points.push_back(pcl::PointXYZ(0.00, -5.25, 0.5));
    inputCloud->points.push_back(pcl::PointXYZ(0.25, -5.25, 0.6));
    inputCloud->points.push_back(pcl::PointXYZ(0.25, -5.50, 0.6));
    inputCloud->points.push_back(pcl::PointXYZ(0.50, -5.50, 0.7));
    inputCloud->points.push_back(pcl::PointXYZ(0.50, -5.75, 0.7));
    inputCloud->points.push_back(pcl::PointXYZ(0.75, -5.75, 0.8));
    inputCloud->points.push_back(pcl::PointXYZ(0.75, -6.00, 0.8));
    inputCloud->points.push_back(pcl::PointXYZ(1.00, -6.00, 0.9));
    inputCloud->points.push_back(pcl::PointXYZ(1.00, -6.25, 0.9));

    inputCloud->points.push_back(pcl::PointXYZ(-0.50, -5.50, 1.0));
    inputCloud->points.push_back(pcl::PointXYZ(-0.50, -5.75, 1.0));
    inputCloud->points.push_back(pcl::PointXYZ(-0.25, -5.75, 1.1));
    inputCloud->points.push_back(pcl::PointXYZ(-0.25, -6.00, 1.1));
    inputCloud->points.push_back(pcl::PointXYZ(-0.00, -6.00, 1.2));
    inputCloud->points.push_back(pcl::PointXYZ(-0.00, -6.25, 1.2));
    inputCloud->points.push_back(pcl::PointXYZ( 0.25, -6.25, 1.3));
    inputCloud->points.push_back(pcl::PointXYZ( 0.25, -6.50, 1.3));
    inputCloud->points.push_back(pcl::PointXYZ( 0.50, -6.50, 1.4));
    inputCloud->points.push_back(pcl::PointXYZ( 0.50, -6.75, 1.4));

    inputCloud->points.push_back(pcl::PointXYZ(-0.50, -5.50, 0.5));
    inputCloud->points.push_back(pcl::PointXYZ(-0.50, -5.75, 0.5));
    inputCloud->points.push_back(pcl::PointXYZ(-0.25, -5.75, 0.6));
    inputCloud->points.push_back(pcl::PointXYZ(-0.25, -6.00, 0.6));
    inputCloud->points.push_back(pcl::PointXYZ(-0.00, -6.00, 0.7));
    inputCloud->points.push_back(pcl::PointXYZ(-0.00, -6.25, 0.7));
    inputCloud->points.push_back(pcl::PointXYZ( 0.25, -6.25, 0.8));
    inputCloud->points.push_back(pcl::PointXYZ( 0.25, -6.50, 0.8));
    inputCloud->points.push_back(pcl::PointXYZ( 0.50, -6.50, 0.9));
    inputCloud->points.push_back(pcl::PointXYZ( 0.50, -6.75, 0.9));

    //renderRays(viewer, lidar->position, inputCloud);
    //renderPointCloud(viewer, inputCloud, "lidar");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor; // Instantiate on the stack
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 1000, 0.17);
    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 250);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 1, 0), Color(0, 1, 1), Color(1, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId % colors.size()]);

        //Box box = pointProcessor.BoundingBox(cluster);
        BoxQ box = pointProcessor.BoundingBoxQ(cluster);
        renderBox(viewer, box, clusterId);

        clusterId++;
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
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    simpleHighway(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}