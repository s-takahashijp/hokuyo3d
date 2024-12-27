#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class imuRCFilter{
    private:
        ros::NodeHandle _nh;
        ros::NodeHandle _nhPrivate;
        ros::Subscriber _sub_imu;
        ros::Publisher _pub_imu;

    public:
        imuRCFilter();
        ~imuRCFilter();
        sensor_msgs::Imu _last_data;
        bool is_first;
        void imu_cb(const sensor_msgs::Imu msgs);
};

imuRCFilter::imuRCFilter()
    : _nhPrivate("~")
{
    ROS_INFO("imu_RC_filter_node started.");
    is_first = true;
    _sub_imu = _nh.subscribe("/hokuyo3d3/imu", 10, &imuRCFilter::imu_cb, this);
    _pub_imu = _nh.advertise<sensor_msgs::Imu>("/hokuyo3d3/RC_filtered_imu", 10);
}

imuRCFilter::~imuRCFilter()
{
    ROS_INFO("end of program.");
}


void imuRCFilter::imu_cb(const sensor_msgs::Imu msgs)
{
    // 最初だけメッセージの
    if(is_first == true){
        _last_data = msgs;
        is_first = false;
        return;
    }

    // RCフィルタの係数
    float a = 0.999000999;
    float b = 1.0 - a;
    sensor_msgs::Imu data = msgs;

    // RCフィルタ実行部
    // 現在の出力値 = 係数 x 一つ時刻前の出力値 + (1 - 係数) x センサ値
    data.linear_acceleration.x = a * _last_data.linear_acceleration.x + b * msgs.linear_acceleration.x;
    data.linear_acceleration.y = a * _last_data.linear_acceleration.y + b * msgs.linear_acceleration.y;
    data.linear_acceleration.z = a * _last_data.linear_acceleration.z + b * msgs.linear_acceleration.z;

    data.angular_velocity.x = a * _last_data.angular_velocity.x + b * msgs.angular_velocity.x;
    data.angular_velocity.y = a * _last_data.angular_velocity.y + b * msgs.angular_velocity.y;
    data.angular_velocity.z = a * _last_data.angular_velocity.z + b * msgs.angular_velocity.z;

    _last_data = data;

    _pub_imu.publish(data);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "imu_RC_filter_node");
    imuRCFilter imu_RC_filter;
    ros::spin();
    return 0;
}

