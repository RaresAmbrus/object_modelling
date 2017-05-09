#include <registration_services/RegistrationServiceWithReference.h>
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
    string reference_folder;

    if (argc > 2){
        reference_folder = argv[1];
        view_folder = argv[2];
    } else {
        ROS_ERROR_STREAM("Please specify reference folder and view folder.");
        return -1;
    }

    // load PCDs from folder
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
        view_pcds.push_back(cloud);
    }
    ROS_INFO_STREAM("Done loading data.");


    ros::init(argc, argv, "rgbd_view_registration_server_with_reference");
    ros::NodeHandle n;

    /************************************************ TEST SERVICE ********************************************************/
    ros::ServiceClient client = n.serviceClient<registration_services::RegistrationServiceWithReference>("rgbd_view_registration_server_with_reference");
    registration_services::RegistrationServiceWithReference srv;
    vector<tf::Transform> registered_transforms;
    vector<tf::Transform> reference_transforms;

    // copy object data here
    for (size_t i=0; i<reference_pcds.size(); i++){
        // get pose -> ASSUMING IT IS SET IN THE POINT CLOUD
        Eigen::Matrix3d basis(reference_pcds[i]->sensor_orientation_.cast<double>());
        Eigen::Vector3d origin = reference_pcds[i]->sensor_origin_.cast<double>().head<3>();
        tf::Matrix3x3 rot;
        tf::Matrix3x3 identity;identity.setIdentity();
        tf::Vector3 pos;
        tf::matrixEigenToTF(basis, rot);
        tf::vectorEigenToTF(origin, pos);
        tf::Transform tr;
        tr.setBasis(rot);
        tr.setOrigin(pos);
        reference_transforms.push_back(tr);
        geometry_msgs::Transform tr_msg;
        tf::transformTFToMsg(tr, tr_msg);
        srv.request.reference_views_odometry_transforms.push_back(tr_msg);

        // reset transform (OTHERWISE IT MESSES UP THE REGISTRATION)
        reference_pcds[i]->sensor_orientation_ = Eigen::Quaternionf(1.0,0.0,0.0,0.0);
        reference_pcds[i]->sensor_origin_ = Eigen::Vector4f(0.0,0.0,0.0,0.0);

        sensor_msgs::PointCloud2 cloud_msg;
        CloudPtr temp_cloud(new Cloud);
        *temp_cloud = *reference_pcds[i];
        pcl::toROSMsg(*temp_cloud, cloud_msg);
        srv.request.reference_views.push_back(cloud_msg);
    }

    for (size_t i=0; i<view_pcds.size(); i++){

        // reset transform (just in case it's already been set)
        view_pcds[i]->sensor_orientation_ = Eigen::Quaternionf(1.0,0.0,0.0,0.0);
        view_pcds[i]->sensor_origin_ = Eigen::Vector4f(0.0,0.0,0.0,0.0);

        sensor_msgs::PointCloud2 cloud_msg;
        CloudPtr temp_cloud(new Cloud);
        *temp_cloud = *view_pcds[i];

        pcl::toROSMsg(*temp_cloud, cloud_msg);
        srv.request.rgbd_views.push_back(cloud_msg);
    }


    ROS_INFO_STREAM("Testing rgbd_view_registration_server_with_reference service");
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
        ROS_ERROR("Failed to call service rgbd_view_registration_server_with_reference");
        return -1;
    }

    // check registration
    ROS_INFO_STREAM("Visualizing registration");
    pcl::visualization::PCLVisualizer* pg = new pcl::visualization::PCLVisualizer (argc, argv, "test_rgbd_view_registration");
    pg->addCoordinateSystem(1.0);
    int constraint_threshold = 20;

    // check registration
    ROS_INFO_STREAM("Visualizing registration");
    for (size_t i=0; i<reference_pcds.size();i++){
            CloudPtr transformedCloud1(new Cloud);
            pcl_ros::transformPointCloud(*reference_pcds[i], *transformedCloud1, reference_transforms[i]);
//            *transformedCloud1 = *reference_pcds[i];
            stringstream ss; ss<<"Cloud";ss<<i;
            pg->addPointCloud(transformedCloud1, ss.str());
    }

    for (size_t i=0; i<registered_transforms.size();++i){
        CloudPtr transformedCloud1(new Cloud);
        *transformedCloud1 = *view_pcds[i];
        // reset transform (just in case it's already been set)
        transformedCloud1->sensor_orientation_ = Eigen::Quaternionf(1.0,0.0,0.0,0.0);
        transformedCloud1->sensor_origin_ = Eigen::Vector4f(0.0,0.0,0.0,0.0);

        pcl_ros::transformPointCloud(*transformedCloud1,
                                     *transformedCloud1,
                                     registered_transforms[i]);

        stringstream ss; ss<<"Cloud";ss<<(i+reference_pcds.size());
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red(transformedCloud1, 255, 0, 0);
        pg->addPointCloud(transformedCloud1, red, ss.str());

        // add registered pose and save
        Eigen::Vector3d origin; tf::vectorTFToEigen(registered_transforms[i].getOrigin(), origin);
        Eigen::Matrix3d basis; tf::matrixTFToEigen(registered_transforms[i].getBasis(), basis);
        Eigen::Vector4f sensor_origin; sensor_origin(3) = 1.0;
        sensor_origin.head<3>() = origin.cast<float>();
        Eigen::Quaternionf sensor_basis(basis.cast<float>());
        view_pcds[i]->sensor_origin_ = sensor_origin;
        view_pcds[i]->sensor_orientation_ = sensor_basis;
        pcl::io::savePCDFileBinaryCompressed(view_pcd_files[i],*view_pcds[i]);
        ROS_INFO_STREAM("Saving registered transform for cloud "<<view_pcd_files[i]);


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
