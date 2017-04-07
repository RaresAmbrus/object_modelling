#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <geometry_msgs/Transform.h>
#include <pcl_ros/transforms.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/subscriber_filter.h>

#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;

ros::Subscriber sub_pc;
ros::Subscriber sub_control;
std::string output_folder = "";
bool save_pcd = false;
int pcd_counter = 0;

using namespace std;

void pcdCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (save_pcd){
        stringstream ss;
        ss << "cloud"<<std::setfill('0')<<std::setw(4)<<pcd_counter<<".pcd";
        string cloud_path = ss.str();
        if (output_folder != ""){
            cloud_path = output_folder + "/" + cloud_path;
        }

        CloudPtr new_cloud(new Cloud());
        pcl::fromROSMsg(*msg, *new_cloud);
        new_cloud->header = pcl_conversions::toPCL(msg->header);
        pcl::io::savePCDFileBinary(cloud_path, *new_cloud);
        ROS_INFO_STREAM("Saving PCD at "<<cloud_path);

        pcd_counter++;
        save_pcd = false;
    }
}

void controlCallback(const std_msgs::String::ConstPtr& msg)
{
    save_pcd = true;
}


int keyPressed();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_grabber");
    ros::NodeHandle n;
    ROS_INFO("rgbd_grabber initialized.");

    string pc_topic = "/camera/depth_registered/points";
    string control_topic = "/teleop/keyinput";

    if (argc>1){
        output_folder = argv[1];
    }
    if (argc>2){
        pc_topic = argv[2];
    }
    if (argc>3){
        control_topic = argv[3];
    }

    sub_pc = n.subscribe(pc_topic.c_str(), 1, pcdCallback);
    sub_control = n.subscribe(control_topic.c_str(), 1, controlCallback);

    ros::Rate loop_rate(10);
    bool loop = true;
    while (ros::ok() && loop){
        ros::spinOnce();
        loop_rate.sleep();
        int ch = keyPressed();

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
            save_pcd = true;
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
