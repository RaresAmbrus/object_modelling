#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/distances.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/Transform.h>
#include <pcl_ros/transforms.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/subscriber_filter.h>

#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>

#include <boost/lexical_cast.hpp>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;


ros::NodeHandle *n;
ros::Subscriber sub_pc;
ros::Publisher debug_pub;
tf::TransformListener* transform_listener;
tf::TransformBroadcaster* tfb;
std::string output_folder = "";
int pcd_counter = 0;
bool acquire_data = false;
std::string world_frame;
std::string cluster_frame;
std::string pointcloud_topic;
float min_cluster_height;
float min_cluster_size;
float max_cluster_size;
tf::StampedTransform object_transform;
bool object_transform_computed;

using namespace std;

void pcdCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (acquire_data){

        // get cloud data
        CloudPtr new_cloud(new Cloud());
        pcl::fromROSMsg(*msg, *new_cloud);
        new_cloud->header = pcl_conversions::toPCL(msg->header);

        // get TF and trasnform to world frame
        tf::StampedTransform transform;


	cout<<"Waiting for transform from "<<world_frame<<" to "<<new_cloud->header.frame_id<<endl;
        transform_listener->waitForTransform(world_frame, new_cloud->header.frame_id,msg->header.stamp, ros::Duration(20.0) );
        transform_listener->lookupTransform(world_frame, new_cloud->header.frame_id,
                                            msg->header.stamp, transform);
	cout<<"Transform received"<<endl;

        CloudPtr transformed_cloud(new Cloud);
        pcl_ros::transformPointCloud(*new_cloud, *transformed_cloud,transform);
        transformed_cloud->header = new_cloud->header;
        transformed_cloud->header.frame_id = world_frame;

        // filter cloud on z axis
        pcl::PassThrough<PointType> pass;
        pass.setInputCloud (transformed_cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (min_cluster_height, 3.0);
        CloudPtr filtered_cloud (new Cloud);
        pass.filter(*filtered_cloud);
        filtered_cloud->header = transformed_cloud->header;
	cout<<"Cloud fitereld on z axis"<<endl;
        
	sensor_msgs::PointCloud2 temp_msg_1;
        pcl::toROSMsg(*filtered_cloud, temp_msg_1);
        temp_msg_1.header = pcl_conversions::fromPCL(filtered_cloud->header);
        debug_pub.publish(temp_msg_1); // publish in the local frame of reference

        // euclidean clustering
        pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
        tree->setInputCloud (filtered_cloud);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointType> ec;
        ec.setClusterTolerance (0.05);
        ec.setMinClusterSize (min_cluster_size);
        ec.setMaxClusterSize (max_cluster_size);
        ec.setSearchMethod (tree);
        ec.setInputCloud (filtered_cloud);
        ec.extract (cluster_indices);

        std::vector<CloudPtr> clusters;

        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            CloudPtr cloud_cluster (new Cloud());
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                cloud_cluster->points.push_back (filtered_cloud->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;


            clusters.push_back(cloud_cluster);

            j++;
        }

        cout<<"Found "<<clusters.size()<<" clusters. Selecting the closest one to the origin of the world frame of ref."<<endl;

	if (clusters.size() == 0){
		acquire_data = false;
		return;
	}


        int best_cl_index = -1;
        float best_dist = std::numeric_limits<float>::max();
        Eigen::Vector4f best_centroid;

        for (int i=0; i<clusters.size(); ++i){
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*clusters[i], centroid);
            Eigen::Vector4f origin(0.0,0.0,0.0,0.0);
            double dist_from_centroid = pcl::distances::l2(centroid,origin);
            if (dist_from_centroid < best_dist){
                best_dist = dist_from_centroid;
                best_cl_index = i;
                best_centroid = centroid;
            }
        }

        CloudPtr best_cluster = clusters[best_cl_index];
        best_cluster->header = filtered_cloud->header;

        cout<<"Selected cluster has "<<best_cluster->points.size()<<". The point cloud is published on topic: /debug_cloud"<<endl;

        // publish TF frame
        object_transform.setIdentity();
        object_transform.setOrigin(tf::Vector3(
                                       best_centroid(0), best_centroid(1), best_centroid(2)));

        object_transform.stamp_ = ros::Time::now();
        object_transform.frame_id_ = world_frame;
        object_transform.child_frame_id_ = cluster_frame;
	object_transform_computed = true;

        sensor_msgs::PointCloud2 temp_msg;
        pcl::toROSMsg(*best_cluster, temp_msg);
        temp_msg.header = pcl_conversions::fromPCL(best_cluster->header);
        debug_pub.publish(temp_msg); // publish in the local frame of reference

	// compute dimensions
	PointType p_min, p_max;
	pcl::getMinMax3D(*best_cluster, p_min, p_max);
	float object_height = p_max.z - p_min.z;
	float object_radius = sqrt((p_max.x - p_min.x)*(p_max.x - p_min.x) + (p_max.y - p_min.y)*(p_max.y - p_min.y));
	cout<<"Object height "<<object_height<<endl;
	cout<<"Object radius "<<object_radius<<endl;
	n->setParam("/object_height", object_height);
	n->setParam("/object_radius", object_radius);

        acquire_data= false;
    }
}

int keyPressed();

void printUsage(){
    std::cout<<"Usage: ./segment_cluster world_frame cluster_frame pointcloud_topic min_cluster_height min_cluster_size max_cluster_size"<<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_grabber");
    n = new ros::NodeHandle();
    ROS_INFO("rgbd_grabber initialized.");

    if (argc<7){
        printUsage();
        exit(0);
    }
    world_frame = argv[1];
    cluster_frame = argv[2];
    pointcloud_topic = argv[3];
    min_cluster_height = boost::lexical_cast<float>(argv[4]);
    min_cluster_size = boost::lexical_cast<float>(argv[5]);
    max_cluster_size = boost::lexical_cast<float>(argv[6]);




    sub_pc = n->subscribe(pointcloud_topic.c_str(), 1, pcdCallback);
    debug_pub = n->advertise<sensor_msgs::PointCloud2>("/debug_cloud",1000);
    transform_listener = new tf::TransformListener(*n, ros::Duration(100));
    tfb = new tf::TransformBroadcaster();

    ros::Rate loop_rate(10);
    bool loop = true;
    object_transform_computed = false;
    while (ros::ok() && loop){
        ros::spinOnce();
        loop_rate.sleep();
        int ch = keyPressed();
	if (object_transform_computed){
		object_transform.stamp_ = ros::Time::now();
		tfb->sendTransform(object_transform);
	}


        switch (ch)
        {
        case 113: // key q
        {
            std::cout<<"------------------------- Exitting program -----------------------------"<<std::endl;
            loop = false;
            break;
        }

        case 115: // key s
        {
            std::cout<<"------------------------- Saving a frame -----------------------------"<<std::endl;
            acquire_data = true;
            break;
        }
        }
    }

    return 0;
}


int keyPressed()
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    // don't echo and don't wait for ENTER
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);

    // make it non-blocking (so we can check without waiting)
    if (0 != fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK))
    {
        return 0;
    }

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    if (0 != fcntl(STDIN_FILENO, F_SETFL, oldf))
    {
        return 0;
    }

    if(ch != EOF)
    {
//        ungetc(ch, stdin);
        return ch;
    }

    return 0;
}
