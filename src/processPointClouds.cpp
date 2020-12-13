// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
//#include "kdtree.h"

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> vox;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered {new pcl::PointCloud<PointT>};

    vox.setInputCloud(cloud);
    vox.setLeafSize(filterRes, filterRes, filterRes);
    vox.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion {new pcl::PointCloud<PointT>};

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-2, -2, -1, 1));
    roof.setMax(Eigen::Vector4f(3, 2, 1, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point : indices)
        inliers->indices.push_back(point);

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

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    for (int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);

    // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>segResult(cloud, cloud);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    std::cout<<"Segmentation begun"<<std::endl;

    while (maxIterations--)
    {
        pcl::PointIndices::Ptr tempInliers(new pcl::PointIndices);
        pcl::PointXYZI point_1 = cloud->points[rand() % cloud->points.size()];
        pcl::PointXYZI point_2 = cloud->points[rand() % cloud->points.size()];
        pcl::PointXYZI point_3 = cloud->points[rand() % cloud->points.size()];

        float A = (point_2.y - point_1.y) * (point_3.z - point_1.z) - (point_2.z - point_1.z) * (point_3.y - point_1.y);
        float B = (point_2.z - point_1.z) * (point_3.x - point_1.x) - (point_2.x - point_1.x) * (point_3.z - point_1.z);
        float C = (point_2.x - point_1.x) * (point_3.y - point_1.y) - (point_2.y - point_1.y) * (point_3.x - point_1.x);
        float D = -(A * point_1.x + B * point_1.y + C * point_1.z);

        // Measure distance between every point and fitted line
        for (int index = 0; index < cloud->points.size(); index++)
        {
            pcl::PointXYZI point_4 = cloud->points[index];
            if ((point_4.x == point_1.x && point_4.y == point_1.y && point_4.z == point_1.z) || (point_4.x == point_2.x && point_4.y == point_2.y && point_4.y == point_2.z) || (point_4.x == point_3.x && point_4.y == point_3.y && point_4.y == point_3.z))
            {
                tempInliers->indices.push_back(index);
                continue;
            }

            float distance = fabs(A * point_4.x + B * point_4.y + C * point_4.z + D) / sqrt(A * A + B * B + C * C);

            // If distance is smaller than threshold count it as inlier
            if (distance < distanceTol)
                tempInliers->indices.push_back(index);
        }

        // Return indicies of inliers from fitted line with most inliers
        if (tempInliers->indices.size() > inliers->indices.size())
            inliers = tempInliers;
    }

    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
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

    for(pcl::PointIndices getIndices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

        for(int index: getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);

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

template <typename PointT>
void ProcessPointClouds<PointT>::Proximity(int ind, typename pcl::PointCloud<PointT>::Ptr cloud, pcl::PointIndices& cluster, std::vector<bool>& processed, kdtree* tree, float distanceTol)
{
    processed[ind] = true;
    cluster.indices.push_back(ind);

    pcl::PointIndices nearbyPointsInd = tree->search(cloud->points[ind], distanceTol);
    for(int indices: nearbyPointsInd.indices)
    {
        if (processed[indices] == false)
            Proximity(indices, cloud, cluster, processed, tree, distanceTol);
    }
}

template <typename PointT>
std::vector<pcl::PointIndices> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, kdtree* tree, float distanceTol)
{

    std::vector<pcl::PointIndices> clusters;
    std::vector<bool> processed(cloud->points.size(), false);

    for(int i=0; i<cloud->points.size(); i++)
    {
        if(processed[i]==true)
            continue;

        pcl::PointIndices cluster;
        Proximity(i, cloud, cluster, processed, tree, distanceTol);
        clusters.push_back(cluster);
    }

    return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::myClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

    kdtree* tree = new kdtree;

    for (int i=0; i<cloud->points.size(); i++)
        tree->insert(cloud->points[i],i);

    std::vector<pcl::PointIndices> clusterIndices;

    clusterIndices = euclideanCluster(cloud, tree, clusterTolerance);

    for(pcl::PointIndices getIndices: clusterIndices)
    {
        if(getIndices.indices.size() > minSize && getIndices.indices.size() < maxSize) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZI>);

            for (int index: getIndices.indices)
                cloudCluster->points.push_back(cloud->points[index]);

            cloudCluster->width = cloudCluster->points.size();
            cloudCluster->height = 1;
            cloudCluster->is_dense = true;

            clusters.push_back(cloudCluster);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
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

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}