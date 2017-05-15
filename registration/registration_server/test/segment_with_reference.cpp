#include <ros/ros.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/distances.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/filesystem.hpp>
#include <tf_conversions/tf_eigen.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;

using namespace std;

namespace fs = ::boost::filesystem;

void get_files(const fs::path& root, const string& ext, vector<string>& ret)
{
    if(!fs::exists(root) || !fs::is_directory(root)) return;

    fs::recursive_directory_iterator it(root);
    fs::recursive_directory_iterator endit;

    while(it != endit)
    {
        if(fs::is_regular_file(*it) && it->path().extension() == ext) ret.push_back((root / it->path().filename()).c_str());
        ++it;

    }

}

vector<CloudPtr> clusterPointCloud(CloudPtr input_cloud, double tolerance, int min_cluster_size, int max_cluster_size);
CloudPtr rebuildCloud(std::vector<CloudPtr> views, std::vector<tf::Transform> transforms);

int main(int argc, char** argv)
{
    string view_folder;
    string reference_folder;

    if (argc > 2){
        reference_folder = argv[1];
        view_folder = argv[2];
    } else {
        ROS_ERROR_STREAM("Please specify reference folder and view folder.");
        return -1;
    }

    // load PCDs from folder
    vector<tf::Transform> view_transforms;
    vector<tf::Transform> reference_transforms;

    vector<string> reference_pcd_files;
    get_files(reference_folder,".pcd",reference_pcd_files);
    if (!reference_pcd_files.size()){
        ROS_ERROR_STREAM("The folder doesn't contain any PCD files.");
        return -1;
    }
    ROS_INFO_STREAM("Found "<<reference_pcd_files.size()<<" reference PCD files.");
    vector<CloudPtr> reference_pcds;
    for (auto pcd_file : reference_pcd_files){
        pcl::PCDReader reader;
        CloudPtr cloud (new Cloud);
        reader.read (pcd_file, *cloud);

        Eigen::Matrix3d basis(cloud->sensor_orientation_.cast<double>());
        Eigen::Vector3d origin = cloud->sensor_origin_.cast<double>().head<3>();
        tf::Matrix3x3 rot;
        tf::Matrix3x3 identity;identity.setIdentity();
        tf::Vector3 pos;
        tf::matrixEigenToTF(basis, rot);
        tf::vectorEigenToTF(origin, pos);
        tf::Transform tr;
        tr.setBasis(rot);
        tr.setOrigin(pos);
        reference_transforms.push_back(tr);
        cloud->sensor_orientation_ = Eigen::Quaternionf(1.0,0.0,0.0,0.0);
        cloud->sensor_origin_ = Eigen::Vector4f(0.0,0.0,0.0,0.0);

        reference_pcds.push_back(cloud);
    }

    vector<string> view_pcd_files;
    get_files(view_folder,".pcd",view_pcd_files);
    if (!view_pcd_files.size()){
        ROS_ERROR_STREAM("The folder doesn't contain any PCD files.");
        return -1;
    }
    ROS_INFO_STREAM("Found "<<view_pcd_files.size()<<" view PCD files.");
    vector<CloudPtr> view_pcds;
    for (auto pcd_file : view_pcd_files){
        pcl::PCDReader reader;
        CloudPtr cloud (new Cloud);
        reader.read (pcd_file, *cloud);

        Eigen::Matrix3d basis(cloud->sensor_orientation_.cast<double>());
        Eigen::Vector3d origin = cloud->sensor_origin_.cast<double>().head<3>();
        tf::Matrix3x3 rot;
        tf::Matrix3x3 identity;identity.setIdentity();
        tf::Vector3 pos;
        tf::matrixEigenToTF(basis, rot);
        tf::vectorEigenToTF(origin, pos);
        tf::Transform tr;
        tr.setBasis(rot);
        tr.setOrigin(pos);
        view_transforms.push_back(tr);
        cloud->sensor_orientation_ = Eigen::Quaternionf(1.0,0.0,0.0,0.0);
        cloud->sensor_origin_ = Eigen::Vector4f(0.0,0.0,0.0,0.0);

        view_pcds.push_back(cloud);
    }
    ROS_INFO_STREAM("Done loading data.");

    pcl::visualization::PCLVisualizer* pg = new pcl::visualization::PCLVisualizer (argc, argv, "test_rgbd_view_registration");
    pg->addCoordinateSystem(1.0);


    // VISUALIZE DATA
    //    for (size_t i=0; i<reference_pcds.size();i++){
    //            CloudPtr transformedCloud1(new Cloud);
    //            pcl_ros::transformPointCloud(*reference_pcds[i], *transformedCloud1, reference_transforms[i]);
    //            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue(transformedCloud1, 0, 0, 255);
    //            stringstream ss; ss<<"Cloud";ss<<i;
    //            pg->addPointCloud(transformedCloud1, blue, ss.str());
    //    }

    //    for (size_t i=0; i<view_pcds.size();i++){
    //            CloudPtr transformedCloud1(new Cloud);
    //            pcl_ros::transformPointCloud(*view_pcds[i], *transformedCloud1, view_transforms[i]);
    //            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red(transformedCloud1, 255, 0, 0);
    //            stringstream ss; ss<<"Cloud_view";ss<<i;
    //            pg->addPointCloud(transformedCloud1, red, ss.str());
    //    }

    // SUBSAMPLE
    pcl::VoxelGrid<PointType> vg;
    vg.setLeafSize (0.01,0.01,0.01);
    CloudPtr subsampled(new Cloud);

    CloudPtr view_cloud  = rebuildCloud(view_pcds, view_transforms);
    CloudPtr original_view_cloud  (new Cloud);
    *original_view_cloud = *view_cloud;
    vg.setInputCloud (view_cloud);
    vg.filter (*subsampled);
    *view_cloud = *subsampled;

    CloudPtr reference_cloud  = rebuildCloud(reference_pcds, reference_transforms);
    vg.setInputCloud (reference_cloud);
    vg.filter (*subsampled);
    *reference_cloud = *subsampled;

    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue(view_cloud, 0, 0, 255);
        pg->addPointCloud(view_cloud, blue, "view_cloud");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red(reference_cloud, 255, 0, 0);
        pg->addPointCloud(reference_cloud, red, "reference_cloud");
    }

    pg->spin();
    pg->removeAllPointClouds();

    // compute the differences
    pcl::SegmentDifferences<PointType> segment;
    segment.setInputCloud(view_cloud);
    segment.setTargetCloud(reference_cloud);
    segment.setDistanceThreshold(0.003);

    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (reference_cloud);
    segment.setSearchMethod(tree);

    CloudPtr diff_cloud(new Cloud);
    segment.segment(*diff_cloud);

    cout<<"Difference cloud has "<<diff_cloud->points.size()<<endl;

    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue(diff_cloud, 0, 0, 255);
        pg->addPointCloud(diff_cloud, blue, "diff");
        pg->spin();
        pg->removeAllPointClouds();
    }

    // find clusters
    vector<CloudPtr> clusters = clusterPointCloud(diff_cloud, 0.05, 300, 5000);
    cout<<"Clusters found "<<clusters.size()<<endl;

    int best_cl_index = -1;
    float best_dist = std::numeric_limits<float>::max();
    Eigen::Vector4f best_centroid;

    for (int i=0; i<clusters.size(); ++i){
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*clusters[i], centroid);
        Eigen::Vector4f origin(0.0,0.0,0.0,0.0);
        double dist_from_centroid = pcl::distances::l2(centroid,origin);
        if ((dist_from_centroid* 0.8 + centroid(0) * 0.2) < best_dist){
            best_dist = dist_from_centroid* 0.5 + centroid(2) * 0.5 ;
            best_cl_index = i;
            best_centroid = centroid;
        }
    }

    CloudPtr best_cluster = clusters[best_cl_index];

    {
        for (int i=0; i<clusters.size(); ++i){

            stringstream ss;
            ss<<"Cluster"<<i;
            pg->addPointCloud(clusters[i], ss.str());
        }

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue(best_cluster, 0, 0, 255);
        pg->addPointCloud(best_cluster, blue, "best_cluster");
        pg->spin();
        pg->removeAllPointClouds();
    }

//    // REGION GROWING
////    CloudPtr new_view_cloud  = rebuildCloud(view_pcds, view_transforms);
    CloudPtr grown_cluster(new Cloud());
//    std::vector<int> nn_indices;
//    std::vector<float> nn_distances;
//    std::vector<int> src_indices;
//    typename pcl::search::KdTree<PointType>::Ptr tree2 (new pcl::search::KdTree<PointType>);
//    tree2->setInputCloud (original_view_cloud);
//    int search_radius = 0.1;

//    // Iterate through the source data set
//    for (int i = 0; i < static_cast<int> (best_cluster->points.size ()); ++i)
//    {
//        if (!isFinite (best_cluster->points[i]))
//            continue;
//        // Search for the closest point in the target data set (number of neighbors to find = 1)

//        if (!tree2->radiusSearch (best_cluster->points[i], search_radius, nn_indices, nn_distances))
////        if (!tree2->nearestKSearch (best_cluster->points[i], 10, nn_indices, nn_distances))
//        {
//            PCL_WARN ("No neighbor found for point %zu (%f %f %f)!\n", i, best_cluster->points[i].x, best_cluster->points[i].y, best_cluster->points[i].z);
//            continue;
//        }

//        for (int j=0; j<nn_indices.size(); ++j){
//            grown_cluster->push_back(original_view_cloud->points[j]);
//        }
//    }

    // DIFFERENCE TO GROW THE CLUSTER
    {
        pcl::SegmentDifferences<PointType> segment;
        pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);

        segment.setInputCloud(original_view_cloud);
        segment.setTargetCloud(best_cluster);
        tree->setInputCloud (best_cluster);
        segment.setDistanceThreshold(0.004);
        CloudPtr view_without_cl(new Cloud);
        segment.segment(*view_without_cl);

        segment.setTargetCloud(view_without_cl);
        tree->setInputCloud (view_without_cl);
        segment.setDistanceThreshold(0.0005);
        segment.segment(*grown_cluster);
    }

    // VISUALIZE GROWN CLUSTER

    {

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green(original_view_cloud, 0, 255, 0);
        pg->addPointCloud(original_view_cloud, green, "view_cloud");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> blue(best_cluster, 0, 0, 255);
        pg->addPointCloud(best_cluster, blue, "best_cluster");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red(grown_cluster, 255, 0, 0);
        pg->addPointCloud(grown_cluster, red, "grown_cluster");

        pg->spin();
        pg->removeAllPointClouds();
    }

    // save best cluster
    string best_cl_path = view_folder + "/best_cluster.pcd";
    pcl::io::savePCDFileBinaryCompressed(best_cl_path,*grown_cluster);



}




CloudPtr rebuildCloud(std::vector<CloudPtr> views, std::vector<tf::Transform> transforms){
    CloudPtr mergedCloud(new Cloud);

    for (size_t i=0; i<views.size(); i++)
    {
        Cloud transformed_cloud;
        pcl_ros::transformPointCloud(*views[i], transformed_cloud,transforms[i]);
        *mergedCloud+=transformed_cloud;
    }
    return mergedCloud;
}

vector<CloudPtr> clusterPointCloud(CloudPtr input_cloud, double tolerance, int min_cluster_size, int max_cluster_size)
{
    pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
    tree->setInputCloud (input_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance (tolerance);
    ec.setMinClusterSize (min_cluster_size);
    ec.setMaxClusterSize (max_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (input_cloud);
    ec.extract (cluster_indices);

    std::vector<CloudPtr> toRet;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        CloudPtr cloud_cluster (new Cloud());
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (input_cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;


        toRet.push_back(cloud_cluster);

        j++;
    }

    return toRet;
}
