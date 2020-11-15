#include <sstream>
#include <fstream>
#include <turtlesim/Pose.h>

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <pr2_picknplace_msgs/PickPlaceAction.h>
#include <pr2_picknplace_msgs/PicknPlaceGoal.h>
#include <actionlib/client/simple_action_client.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>

class PR2Collect {
  private:
    ros::NodeHandle* nh_;

    geometry_msgs::Pose target_pose;
    geometry_msgs::Pose tempPose;
    std::vector<geometry_msgs::Pose> poseVector;
    geometry_msgs::TransformStamped transformStampOS2;
    geometry_msgs::TransformStamped transformStampPR2;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    cv::Mat storedImage;

    pr2_picknplace_msgs::PickPlaceGoal pr2Pose;

    int counter = 0;
    double roll, pitch, yaw;
    double restrictionXP = 0.78; //0.78
    double restrictionYP = 0.2; //0.4
    double restrictionZP = 0.4; //0.9
    double restrictionXN = 0.8; //0.7
    double restrictionYN = -0.2; //-0.8
    double restrictionZN = 0;
    double subjectX = 1.3;//1.8;//5;
    double subjectY = 0;//0;
    double subjectZ = 0;//0;
    double sphereRad = 0.5;
    double xStep = 0.1;
    double yStep = 0.1;
    double threshold = 0.1;
    float vx = 0, vy = 0, vz = 0;


    std::string poseFileNameTFV = "/home/carson/test_ws/pr2tfv.txt";
    std::string poseFileNameTFQ = "/home/carson/test_ws/pr2tfq.txt";
    std::string poseFileNameV = "/home/carson/test_ws/pr2PoseOutV.txt";
    std::string poseFileNameQ = "/home/carson/test_ws/pr2PoseOutQ.txt";
    std::string poseFileNameOS2V = "/home/carson/test_ws/os2PoseOutV.txt";
    std::string poseFileNameOS2Q = "/home/carson/test_ws/os2PoseOutQ.txt";
    std::ofstream pr2PoseOutQ, pr2PoseOutV, pr2PoseOutTFQ, pr2PoseOutTFV,
        os2PoseOutTFV, os2PoseOutTFQ;

    ros::Subscriber getImageSub;
    ros::Subscriber getPoseSub;

  public:
    PR2Collect(ros::NodeHandle* nh);

    void loadParams();
    void init();
    void rosSetup();
    void generateGrid();

    void getImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void getPoseCallback(const geometry_msgs::Pose& msg);
    void collectImage();
    void savePose();

    void runRobot();
};

PR2Collect::PR2Collect(ros::NodeHandle* nh) : tfListener(tfBuffer), nh_(nh) {
    this->rosSetup();
}

void PR2Collect::rosSetup() {
    getImageSub = nh_->subscribe("/usb_cam/image_raw", 1000,
                                 &PR2Collect::getImageCallback, this);
    //FINISH
    //getPoseSub = nh_->subscribe()

    //Open files
    //pr2PoseOutV.open(poseFileNameV);
    //pr2PoseOutQ.open(poseFileNameQ);
    ROS_INFO_STREAM("in ros setup");
    pr2PoseOutTFV.open(poseFileNameTFV);
    pr2PoseOutTFQ.open(poseFileNameTFQ);
    os2PoseOutTFV.open(poseFileNameOS2V);
    os2PoseOutTFQ.open(poseFileNameOS2Q);
}

void PR2Collect::init() {

}

void PR2Collect::savePose() {
    pr2PoseOutQ << transformStampPR2.transform.rotation.x << "\n";
    pr2PoseOutQ << transformStampPR2.transform.rotation.y << "\n";
    pr2PoseOutQ << transformStampPR2.transform.rotation.z << "\n";
    pr2PoseOutQ << transformStampPR2.transform.rotation.w << "\n";

    pr2PoseOutV << transformStampPR2.transform.translation.x << "\n";
    pr2PoseOutV << transformStampPR2.transform.translation.y << "\n";
    pr2PoseOutV << transformStampPR2.transform.translation.z << "\n";

    os2PoseOutTFQ << transformStampOS2.transform.rotation.x << "\n";
    os2PoseOutTFQ << transformStampOS2.transform.rotation.y << "\n";
    os2PoseOutTFQ << transformStampOS2.transform.rotation.z << "\n";
    os2PoseOutTFQ << transformStampOS2.transform.rotation.w << "\n";

    os2PoseOutTFV << transformStampOS2.transform.translation.x << "\n";
    os2PoseOutTFV << transformStampOS2.transform.translation.y << "\n";
    os2PoseOutTFV << transformStampOS2.transform.translation.z << "\n";
}

/*
void PR2Collect::getPoseCallback(const geometry_msgs::Pose &msg){
    tempPose.position.x = msg.position.x;
    tempPose.position.y = msg.position.y;
    tempPose.position.z = msg.position.z;
    tempPose.orientation.x = msg.orientation.x;
    tempPose.orientation.y = msg.orientation.y;
    tempPose.orientation.z = msg.orientation.z;
    tempPose.orientation.w = msg.orientation.w;
}
*/
void PR2Collect::getImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    //Copy the ros image to cv::Mat

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        storedImage = cv_ptr->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void PR2Collect::collectImage() {
    ROS_INFO_STREAM("Saving image");

    std::string fileLocation = "/home/carson/";
    std::string fileEnd = ".jpg";
    fileLocation += std::to_string(counter) + fileEnd;

    cv::imwrite(fileLocation, storedImage);
    counter++;
}

void PR2Collect::runRobot() {
    ROS_INFO_STREAM("In runRobot");

    actionlib::SimpleActionClient<pr2_picknplace_msgs::PickPlaceAction>
    ac("/pr2_picknplace_right/pr2_picknplace", true);

    ROS_INFO_STREAM("Waiting for server");
    ac.waitForServer(); // UNCOMMENT THIS
    ROS_INFO_STREAM("Server returned");

    pr2Pose.goal.request = 2;  // MOVETO request
    pr2Pose.goal.header.frame_id = "base_link";  // Frame of reference

    target_pose.position.x = 0.7;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.7;
    target_pose.orientation.x = 0;
    target_pose.orientation.y = -0.707;
    target_pose.orientation.z = 0;
    target_pose.orientation.w = 0.707;

    target_pose.position.y += 0.05;
    pr2Pose.goal.object_pose = target_pose;
    
    ROS_INFO_STREAM("Sending Goal");
    ac.sendGoal(pr2Pose);
    ac.waitForResult(ros::Duration(10.0)); // UNCOMMENT THIS
    ROS_INFO_STREAM("Sent goal");

    try {
        transformStampPR2 = tfBuffer.lookupTransform("base_link",
                                                     "r_gripper_tool_frame", ros::Time(0));
        //transformStampOS2 = tfBuffer.lookupTransform("ptam_world", "ps3_cam", ros::Time(0));
        transformStampOS2 = transformStampPR2; //change this!!!
    } catch (tf2::TransformException& ex) {
        //ROS_WARN("%s", ex.what());
    }

    if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        //return 1;
        ROS_INFO("SUCCEEDED\n");
    }
    collectImage();
    savePose();

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "grid_move_too_node");
    ros::NodeHandle nh;
    PR2Collect pr2Instance(&nh);
    //ros::Subscriber subImage = nh.subscribe("/usb_cam/image_raw", 1, &PR2Collect::getImageCallback, &pr2Instance);

    //Might want this to go in the while loop...
    pr2Instance.runRobot();

    ros::Rate rate(200.0);

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
