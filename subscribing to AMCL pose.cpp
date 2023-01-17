//subscribing to AMCL pose

#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
double poseAMCLx, poseAMCLy, poseAMCLa;
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{                   //const 필수,  geometry_msgs파일의 PoseWithCovarianceStamped.msg 에서 고정 포인터로 msgAMCL변수를 선언
    poseAMCLx = msgAMCL->pose.pose.position.x;
    poseAMCLy = msgAMCL->pose.pose.position.y;
    poseAMCLa = msgAMCL->pose.pose.orientation.w;   
    //ROS_INFO(msgAMCL);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "err_eval");
    ros::NodeHandle n;
    ros::Subscriber sub_amcl = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", poseAMCLCallback);
    //                                                                       amcl_pose 토픽에서 poseAMCLCallback에서 선언한 포인터로 이동
    ros::Rate loop_rate(10);
    ros::spinOnce();

    int count = 0;
    while (ros::ok())
    {

        geometry_msgs::Pose error;
        error.position.x = poseAMCLx;
        error.position.y = poseAMCLy;
        error.orientation.w = poseAMCLa;
        pub.publish(error);

        ros::spinOnce();
        loop_rate.sleep();
        count=count+1;
    }

    return 0;
}
