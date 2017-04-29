#include <registration_services/RegistrationService.h>
#include <ros/ros.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
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

CloudPtr rebuildCloud(std::vector<CloudPtr> views, std::vector<tf::StampedTransform> transforms);

int main(int argc, char** argv)
{
    string view_folder;

    if (argc > 1){
        view_folder = argv[1];
    } else {
        ROS_ERROR_STREAM("Please specify folder with PCDs.");
        return -1;
    }

    // load PCDs from folder
    vector<string> pcd_files;
    get_files(view_folder,".pcd",pcd_files);
    if (!pcd_files.size()){
        ROS_ERROR_STREAM("The folder doesn't contain any PCD files.");
        return -1;
    }
    ROS_INFO_STREAM("Found "<<pcd_files.size()<<" PCD files.");
    vector<CloudPtr> pcds;
    for (auto pcd_file : pcd_files){
        pcl::PCDReader reader;
        CloudPtr cloud (new Cloud);
        reader.read (pcd_file, *cloud);
        pcds.push_back(cloud);
    }
    ROS_INFO_STREAM("Done loading data.");


    ros::init(argc, argv, "test_rgbd_view_registration_service");
    ros::NodeHandle n;

    /************************************************ TEST SERVICE ********************************************************/
    ros::ServiceClient client = n.serviceClient<registration_services::RegistrationService>("rgbd_view_registration_server");
    registration_services::RegistrationService srv;
    vector<tf::Transform> registered_transforms;

    // copy object data here
    for (size_t i=0; i<pcds.size(); i++){

        // reset transform (just in case it's already been set)
        pcds[i]->sensor_orientation_ = Eigen::Quaternionf(1.0,0.0,0.0,0.0);
        pcds[i]->sensor_origin_ = Eigen::Vector4f(0.0,0.0,0.0,0.0);

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*pcds[i], cloud_msg);
        srv.request.rgbd_views.push_back(cloud_msg);
    }
    srv.request.rgbd_views_odometry_transforms.clear();

    ROS_INFO_STREAM("Testing rgbd_view_registration_server service");
    if (client.call(srv))
    {
        int total_constraints = 0;
        std::for_each(srv.response.rgbd_view_correspondences.begin(), srv.response.rgbd_view_correspondences.end(), [&] (int n) {
            total_constraints += n;
            std::cout<<"Num constraints "<<n<<std::endl;
        });

        ROS_INFO_STREAM("Registration done. Number of rgbd view registration constraints "<<total_constraints<<". Number of rgbd view transforms "<<srv.response.rgbd_view_transforms.size());

        if (total_constraints <= 0){
            ROS_INFO_STREAM("RGBD view Registration unsuccessful due to insufficient constraints.");
            return -1;
        }
        for (auto tr : srv.response.rgbd_view_transforms){
            tf::Transform transform;
            tf::transformMsgToTF(tr, transform);
            registered_transforms.push_back(transform);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service rgbd_view_registration_server");
        return -1;
    }

    // check registration
    ROS_INFO_STREAM("Visualizing registration");
    pcl::visualization::PCLVisualizer* pg = new pcl::visualization::PCLVisualizer (argc, argv, "test_rgbd_view_registration");
    pg->addCoordinateSystem(1.0);
    int constraint_threshold = 20;

    // check registration
    ROS_INFO_STREAM("Visualizing registration");
    for (size_t i=0; i<pcds.size();i++){
        if (srv.response.rgbd_view_correspondences[i] >= constraint_threshold){
            CloudPtr transformedCloud1(new Cloud);
            *transformedCloud1 = *pcds[i];
            pcl_ros::transformPointCloud(*transformedCloud1, *transformedCloud1,registered_transforms[i]);

            stringstream ss; ss<<"Cloud";ss<<i;
            pg->addPointCloud(transformedCloud1, ss.str());

            // add registered pose and save
            Eigen::Vector3d origin; tf::vectorTFToEigen(registered_transforms[i].getOrigin(), origin);
            Eigen::Matrix3d basis; tf::matrixTFToEigen(registered_transforms[i].getBasis(), basis);
            Eigen::Vector4f sensor_origin; sensor_origin(3) = 1.0;
            sensor_origin.head<3>() = origin.cast<float>();
            Eigen::Quaternionf sensor_basis(basis.cast<float>());
            pcds[i]->sensor_origin_ = sensor_origin;
            pcds[i]->sensor_orientation_ = sensor_basis;
            pcl::io::savePCDFileBinaryCompressed(pcd_files[i],*pcds[i]);
            ROS_INFO_STREAM("Saving registered transform for cloud "<<pcd_files[i]);
        }
    }

    pg->spin();
    pg->removeAllPointClouds();



}




CloudPtr rebuildCloud(std::vector<CloudPtr> views, std::vector<tf::StampedTransform> transforms){
    CloudPtr mergedCloud(new Cloud);

    for (size_t i=0; i<views.size(); i++)
    {
        Cloud transformed_cloud;
        pcl_ros::transformPointCloud(*views[i], transformed_cloud,transforms[i]);
        *mergedCloud+=transformed_cloud;
    }
    return mergedCloud;
}
