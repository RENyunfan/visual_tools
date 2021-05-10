#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include "poly_traj_utils/poly_visual_utils.hpp"
#include "pthread.h"
#include <tf/transform_broadcaster.h>

pthread_mutex_t mutex;
bool has_odom = false;
class ScopeLock{
public:
    ScopeLock(pthread_mutex_t _mutex) : mutex(_mutex) {
        pthread_mutex_lock(&mutex);
    }
    ~ScopeLock() {
        pthread_mutex_unlock(&mutex);
    }
private:
    pthread_mutex_t mutex;
};
using namespace std;
using namespace Eigen;
typedef Matrix<double, 3, 3> Mat33;
typedef Matrix<double, 3, 1> Vec3;
ros::Publisher local_pc_pub;
ros::Subscriber pose_sub, odom_sub;

double deg2rad(double deg){
    return deg/180.0*M_PI;
}
Mat33 R;
Vec3 cur_posi;tf::TransformBroadcaster* broadcaster;
tf::Transform ttt;
void poseCallback(const geometry_msgs::PoseStampedConstPtr & msg){
    has_odom = true;
    ScopeLock t__(mutex);
    Vec3 pose;
    Vector4d q;
    q(0)    = msg->pose.orientation.w;
    q(1)    = msg->pose.orientation.x;
    q(2)    = msg->pose.orientation.y;
    q(3)    = msg->pose.orientation.z;
    pose(0) = msg->pose.position.x;
    pose(1) = msg->pose.position.y;
    pose(2) = msg->pose.position.z;
    cur_posi = MsgUtils::poseToEigenVec(msg->pose);
    R =MsgUtils::poseToEigenRotation(msg->pose);

    ttt.setOrigin(tf::Vector3(pose(0), pose(1), pose(2)));
    ttt.setRotation(tf::Quaternion(q(1), q(2), q(3), q(0)));

}

void TFCallback(const ros::TimerEvent & e){
    if(! has_odom){
        return ;
    }
    broadcaster->sendTransform(tf::StampedTransform(ttt,ros::Time::now(), string("world"),  string("third_view")));
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "local_sensing");
    ros::NodeHandle nh( "~" );
    tf::TransformBroadcaster b;
    broadcaster = &b;
    pose_sub = nh.subscribe("/mavros/vision_pose/pose", 1,poseCallback);
    ros::Timer view_timer = nh.createTimer(ros::Duration(0.01), TFCallback);
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
