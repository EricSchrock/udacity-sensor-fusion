// PCL lib Functions for processing point clouds 

#include <unordered_set>
#include "processPointClouds.h"
#include <pcl/filters/project_inliers.h>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Reduce the point cloud's resolution by removing all but one point per voxel
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);

    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    // Crop the point cloud to a boxed region within a certain distance around the host vehicle
    pcl::CropBox<PointT> region(true);
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);

    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    // Remove reflections off the host vehicle
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);

    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1.0, 1.0));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1.0));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};

    for (int point : indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    typename pcl::PointCloud<PointT>::Ptr obstaclesCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    for (int index : inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstaclesCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstaclesCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};

    // My RANSAC implementation (took 42 ms on the example data set compared to 17 ms for the built inPCL RANSAC function)
    while (maxIterations--)
    {
        std::unordered_set<int> inliersTemp; // Set so no duplicate values

        while (inliersTemp.size() < 3)
        {
            inliersTemp.insert(rand() % (cloud->points.size()));
        }

        float x1, y1, z1;
        float x2, y2, z2;
        float x3, y3, z3;

        auto iterator = inliersTemp.begin();
        x1 = cloud->points[*iterator].x;
        y1 = cloud->points[*iterator].y;
        z1 = cloud->points[*iterator].z;

        iterator++;
        x2 = cloud->points[*iterator].x;
        y2 = cloud->points[*iterator].y;
        z2 = cloud->points[*iterator].z;

        iterator++;
        x3 = cloud->points[*iterator].x;
        y3 = cloud->points[*iterator].y;
        z3 = cloud->points[*iterator].z;

        float A = ((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1));
        float B = ((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1));
        float C = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1));
        float D = -((A * x1) + (B * y1) + (C * z1));

        for (int index = 0; index < cloud->points.size(); index++)
        {
            if (inliersTemp.count(index) > 0)
            {
                continue;
            }

            float x = cloud->points[index].x;
            float y = cloud->points[index].y;
            float z = cloud->points[index].z;

            float d = fabs((A * x) + (B * y) + (C * z) + D) / sqrt((A * A) + (B * B) + (C * C));

            if (d <= distanceThreshold)
            {
                inliersTemp.insert(index);
            }
        }

        if (inliersTemp.size() > inliers->indices.size())
        {
            inliers->indices.clear();

            for (const auto& elem : inliersTemp)
            {
                inliers->indices.push_back(elem);
            }
        }
    }

    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for (pcl::PointIndices getIndices : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

        for (int index : getIndices.indices)
        {
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Project the cluster onto the XY plane
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr clusterXYProjection(new pcl::PointCloud<pcl::PointXYZ>);

    for (PointT point : cluster->points)
    {
        clusterXYProjection->points.push_back(pcl::PointXYZ(point.x, point.y, 0));
    }

    // Code based on http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
    // Compute principle directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*clusterXYProjection, pcaCentroid);

    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*clusterXYProjection, pcaCentroid, covariance);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    // Transform the cluster so the principle components correspond with the origin and axes
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3, 1>(0, 3) = -1.0f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());

    typename pcl::PointCloud<PointT>::Ptr clusterProjected(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *clusterProjected, projectionTransform);

    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*clusterProjected, minPoint, maxPoint);

    const Eigen::Vector3f meanDiagonal = 0.5f * (maxPoint.getVector3fMap() + minPoint.getVector3fMap());
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA);

    BoxQ boxq;
    boxq.bboxQuaternion = bboxQuaternion;
    boxq.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    boxq.cube_length = maxPoint.x - minPoint.x;
    boxq.cube_width = maxPoint.y - minPoint.y;
    boxq.cube_height = maxPoint.z - minPoint.z;

    return boxq;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) // load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }

    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}
