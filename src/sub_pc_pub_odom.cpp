#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>


class SubpcPubodom{
    private:
        ros::NodeHandle _nh;
        ros::NodeHandle _nhPrivate;
        ros::Subscriber _sub_odom;
        ros::Subscriber _sub_pc;
        ros::Publisher _pub_odom;

        nav_msgs::Odometry odom; // 新しいデータを更新する。

    public:
        SubpcPubodom();
        void callbackOdom(const nav_msgs::Odometry msgs);
        void callbackPC(const sensor_msgs::PointCloud2 msgs);
        
};

SubpcPubodom::SubpcPubodom()
    : _nhPrivate("~")
{
    _sub_odom = _nh.subscribe("/hokuyo_lio/lidar_odom", 10, &SubpcPubodom::callbackOdom, this);
    _sub_pc = _nh.subscribe("/hokuyo_lio/aligned_scan_points", 10, &SubpcPubodom::callbackPC, this);
    _pub_odom = _nh.advertise<sensor_msgs::PointCloud2>("/hokuyo_lio/sync_odom", 10);
}

void SubpcPubodom::callbackOdom(const nav_msgs::Odometry msgs)
{
    odom = msgs;
}

void SubpcPubodom::callbackPC(const sensor_msgs::PointCloud2 msgs)
{
    _pub_odom.publish(odom);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "sub_pc_pub_odom");
    SubpcPubodom SubpcPubodom;
    ros::spin();
    return 0;
}

