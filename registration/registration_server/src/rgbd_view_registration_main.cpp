#include <ros/ros.h>
#include <registration_services/RegistrationService.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/tf.h>
#include <geometry_msgs/Transform.h>
#include <pcl_ros/transforms.h>
#include "rgbd_view_registration_server/rgbd_view_registration_optimizer.h"

typedef pcl::PointXYZRGB PointType;

using namespace std;

bool optimize(const vector<boost::shared_ptr<pcl::PointCloud<PointType>>>& rgbd_views,
              const vector<tf::StampedTransform>& rgbd_views_odometry_transforms,
              std::vector<int>& rgbd_view_constraints,
              vector<tf::StampedTransform>& rgbd_view_registered_transforms);

void publishRegistrationResult(const vector<boost::shared_ptr<pcl::PointCloud<PointType>>>& rgbd_views,
                               const vector<tf::StampedTransform>& rgbd_views_transforms);
ros::Publisher pubRegistrationResult;

bool rgbd_view_registration_service(
        registration_services::RegistrationService::Request  &req,
        registration_services::RegistrationService::Response &res)
{
    ROS_INFO("Received an additonal view registration request");
    ROS_INFO_STREAM("Number of AV: "<<req.rgbd_views.size());
    ROS_INFO_STREAM("Number of AV odometry transforms: "<<req.rgbd_views_odometry_transforms.size());

    // check rgbd view data
    if ((!req.rgbd_views.size())){
        // no rgbd views, cannot register
        ROS_ERROR_STREAM("rgbd_view_registration_service: no rgbd views provided. Cannot register.");
        return true;
    }

    if ((req.rgbd_views_odometry_transforms.size() != 0) && (req.rgbd_views_odometry_transforms.size() != req.rgbd_views.size())){
        ROS_ERROR_STREAM("rgbd_view_registration_service: the number of rgbd views provided differs from the number of odometry transforms. Will not use the odometry transforms");
        return true;
    }


    // set up optimization data
    vector<boost::shared_ptr<pcl::PointCloud<PointType>>> rgbd_views;
    vector<tf::StampedTransform> rgbd_views_odometry_transforms;
    vector<boost::shared_ptr<pcl::PointCloud<PointType>>> observation_intermediate_clouds;
    vector<tf::StampedTransform> observation_intermediate_clouds_transforms;

    // object optimization data
    for (auto rgbd_view_msg : req.rgbd_views){
        boost::shared_ptr<pcl::PointCloud<PointType>> rgbd_view ( new pcl::PointCloud<PointType>);
        pcl::fromROSMsg(rgbd_view_msg, *rgbd_view);
        rgbd_views.push_back(rgbd_view);
    }

    if ((req.rgbd_views_odometry_transforms.size() != 0) && (req.rgbd_views_odometry_transforms.size() == req.rgbd_views.size())){
        // get odometry transforms
        for (auto odometry : req.rgbd_views_odometry_transforms){
            tf::Transform view_transform;
            tf::transformMsgToTF(odometry, view_transform);
            tf::StampedTransform view_transform_stamped(view_transform, ros::Time::now(),"",""); // dummy values, not important
            rgbd_views_odometry_transforms.push_back(view_transform_stamped);
        }
    }

    // pass data to the optimizer
    std::vector<int> rgbd_view_constraints;
    vector<tf::StampedTransform> rgbd_view_registered_transforms;
    int observation_constraints;
    tf::StampedTransform observation_transform;

    ROS_INFO_STREAM("Registering rgbd views ...");
    optimize(rgbd_views, rgbd_views_odometry_transforms,
             rgbd_view_constraints, rgbd_view_registered_transforms);

    // return data
    // constraint number
    res.rgbd_view_correspondences.assign(rgbd_view_constraints.begin(), rgbd_view_constraints.end());

    // rgbd view registered transforms
    for (auto reg_tf : rgbd_view_registered_transforms){
        geometry_msgs::Transform registered_transform_msg;
        tf::transformTFToMsg(reg_tf, registered_transform_msg);
        res.rgbd_view_transforms.push_back(registered_transform_msg);
    }

    // publish data (debug)
    publishRegistrationResult(rgbd_views,
                              rgbd_view_registered_transforms);

    return true;
}

bool optimize(const vector<boost::shared_ptr<pcl::PointCloud<PointType>>>& rgbd_views,
              const vector<tf::StampedTransform>& rgbd_views_odometry_transforms,
              std::vector<int>& rgbd_view_constraints,
              vector<tf::StampedTransform>& rgbd_view_registered_transforms){

    bool register_to_observation = false;

    bool verbose = true;
    vector<tf::Transform> registered_transforms;

    RGBDViewRegistrationOptimizer optimizer(verbose);
    optimizer.registerViews<PointType>(rgbd_views,
                                       rgbd_views_odometry_transforms,
                                       rgbd_view_constraints,registered_transforms);

    for (auto tf : registered_transforms){
        tf::StampedTransform tf_stamped(tf, ros::Time::now(),"", "");
        rgbd_view_registered_transforms.push_back(tf_stamped);
    }

    return true;
}

void publishRegistrationResult(const vector<boost::shared_ptr<pcl::PointCloud<PointType>>>& rgbd_views,
                               const vector<tf::StampedTransform>& rgbd_views_transforms){

    boost::shared_ptr<pcl::PointCloud<PointType>> registered_cloud(new pcl::PointCloud<PointType>);

    for (size_t i=0; i<rgbd_views.size();i++){
        boost::shared_ptr<pcl::PointCloud<PointType>> transformedCloud1(new pcl::PointCloud<PointType>);
        pcl_ros::transformPointCloud(*rgbd_views[i], *transformedCloud1,rgbd_views_transforms[i]);

        *registered_cloud+= *transformedCloud1;
    }

    // downsample
    boost::shared_ptr<pcl::PointCloud<PointType>> registered_cloud_downsampled(new pcl::PointCloud<PointType>);
    pcl::VoxelGrid<PointType> vg;
    double leaf_size = 0.03;
    vg.setInputCloud (registered_cloud);
    vg.setLeafSize (leaf_size,leaf_size,leaf_size);
    vg.filter (*registered_cloud_downsampled);

    ROS_INFO_STREAM("Publishing registered view cloud - no points "<<registered_cloud_downsampled->points.size()<<" on topic /rgbd_view_registration/rgbd_view_cloud");
    sensor_msgs::PointCloud2 msg_cloud;
    pcl::toROSMsg(*registered_cloud_downsampled, msg_cloud);
    msg_cloud.header.frame_id = "/map";
    pubRegistrationResult.publish(msg_cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_view_registration_server");
    ros::NodeHandle n;

    ros::ServiceServer service =  n.advertiseService("rgbd_view_registration_server", rgbd_view_registration_service);
    pubRegistrationResult = n.advertise<sensor_msgs::PointCloud2>("/rgbd_view_registration/registered_view_cloud", 1);

    ROS_INFO("rgbd_view_registration_server started.");
    ros::spin();

    return 0;
}

