#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace pcl;
using namespace message_filters;

class CloudAssembler
{
    ros::NodeHandle nh;
    ros::Publisher cloud_pub;
    pcl::PointCloud<pcl::PointXYZRGB> combined_cloud;
    std::string fetch_cloud_topic;
    std::string camera_cloud_topic;

    public:
    CloudAssembler()
    {
        ros::NodeHandle nh;

        ros::Publisher cloud_pub = nh.advertise<PointCloud<PointXYZRGB>>("cloud_assembler", 1);

        ros::param::get("~fetch_cloud_topic", fetch_cloud_topic);
        ros::param::get("~camera_cloud_topic", camera_cloud_topic);

        std::cout << camera_cloud_topic << std::endl;

        message_filters::Subscriber<PointCloud<PointXYZRGB>> fetch_cloud_sub(nh, fetch_cloud_topic, 1);
        message_filters::Subscriber<PointCloud<PointXYZRGB>> camera_cloud_sub(nh, camera_cloud_topic, 1);
        TimeSynchronizer<PointCloud<PointXYZRGB>, PointCloud<PointXYZRGB>> sync(fetch_cloud_sub, camera_cloud_sub, 100);
        sync.registerCallback(boost::bind(&CloudAssembler::callback, this, _1, _2));
    }
    
    
    void callback(const PointCloud<PointXYZRGB>::ConstPtr& fetch_cloud, const PointCloud<PointXYZRGB>::ConstPtr& camera_cloud)
    {
        PointCloud<PointXYZRGB> cld1 = *fetch_cloud;
        PointCloud<PointXYZRGB> cld2 = *camera_cloud;

        combined_cloud = cld1 + cld2;

        std::cout << "Publishing combined point cloud" << std::endl;

        cloud_pub.publish(combined_cloud);
    }
};




int main (int argc, char** argv)
{
    ros::init(argc, argv, "point_clouds_assembler");

    std::cout << "I'm running" << std::endl;

    CloudAssembler assemble;

    ros::spin();

    return 0;
}