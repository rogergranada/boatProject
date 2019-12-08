#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <zed/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <zedpub/Img.h>

using namespace sl::zed;
using namespace std;

int main(int argc, char** argv) {
    // infos of images
    int tstamp = std::time(0);
    string dright = "/media/ubuntu/JetsonSSD/" + to_string(tstamp) + "/right/";
    string nm_right;
    string dleft = "/media/ubuntu/JetsonSSD/" + to_string(tstamp) + "/left/";
    string nm_left;
    string ddepth = "/media/ubuntu/JetsonSSD/" + to_string(tstamp) + "/depth/";
    string nm_depth;

    int cnt = 0;

    // init ROS
    ros::init(argc, argv, "zedpub");

    // get host name to add to the topic
    char hostname[HOST_NAME_MAX];
    gethostname(hostname, HOST_NAME_MAX);

    // node handle to publish
    ros::NodeHandle nh;
    ros::Publisher camera_pub = nh.advertise<zedpub::Img>(string(hostname)+"/camera", 1000);
    ros::Rate loop_rate(10);

    // start ZED camera configuration
    Camera* zed = new Camera(HD720, 30);
    InitParams parameters;
    parameters.verbose = true;
    parameters.mode = PERFORMANCE;
    ERRCODE err = zed->init(parameters);
    if (err != SUCCESS) {
        cout << "Unable to init ZED: " << errcode2str(err) << endl;
        delete zed;
        return 1;
    }

    ZED_SELF_CALIBRATION_STATUS old_self_calibration_status = SELF_CALIBRATION_NOT_CALLED;
    SENSING_MODE dm_type = FILL;
    zed->setDepthClampValue(10000);

    int width = zed->getImageSize().width;
    int height = zed->getImageSize().height;

    cv::Mat imright(height, width, CV_8SC3);
    cv::Mat imleft(height, width, CV_8SC3);
    cv::Mat imdepth(height, width, CV_8UC1);
    cv::Mat imdisp(height, width, CV_8UC3);
 
    int ConfidenceIdx = 100;
    
    while (ros::ok()){

        zed->setConfidenceThreshold(ConfidenceIdx);
        bool res = zed->grab(dm_type); 

        if (!res) {
            if (old_self_calibration_status != zed->getSelfCalibrationStatus()) {
                cout << "Self Calibration Status : " << statuscode2str(zed->getSelfCalibrationStatus()) << endl;
                old_self_calibration_status = zed->getSelfCalibrationStatus();
            }
            
            // get right image
            slMat2cvMat(zed->retrieveImage(SIDE::RIGHT)).copyTo(imright);
            nm_right = dright + to_string(tstamp) + "_" + to_string(cnt) + string(".png");
            //cv::imwrite(nm_right, imright);

            // get left image
            slMat2cvMat(zed->retrieveImage(SIDE::LEFT)).copyTo(imleft);
            nm_left = dleft + to_string(tstamp) + "_" + to_string(cnt) + string(".png");
            //cv::imwrite(nm_left, imleft);

            // get depth image
            sl::zed::Mat depth = zed->retrieveMeasure(MEASURE::DEPTH);  // get the pointer
            slMat2cvMat(zed->normalizeMeasure(MEASURE::DEPTH)).copyTo(imdepth);
            nm_depth = ddepth + to_string(tstamp) + "_" + to_string(cnt) + string(".png");
            //cv::imwrite(nm_depth, imdepth);

            // publish the name of the image
            zedpub::Img msg;
            std::stringstream ss;
            ss << to_string(cnt) + string(".png");
        
            ros::Time time = ros::Time::now();
            msg.header.stamp = time;
            msg.header.frame_id = to_string(cnt);
            msg.imgname = ss.str();
            camera_pub.publish(msg);

            cnt++;

        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return 0;
}
