#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <zed/Camera.hpp>
#include <opencv2/opencv.hpp>

using namespace sl::zed;
using namespace std;

int main(int argc, char** argv) {
    // infos of images
    int tstamp = std::time(0);
    string dright = "/media/ubuntu/Card/right/";
    string nm_right;
    string dleft = "/media/ubuntu/Card/left/";
    string nm_left;
    string ddepth = "/media/ubuntu/Card/depth/";
    string nm_depth;

    int cnt = 0;

    // init ROS
    ros::init(argc, argv, "zedpub");

    // node handle to publish
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_left = it.advertise("camera/left_image", 1);
    sensor_msgs::ImagePtr msg_left;
    image_transport::Publisher pub_right = it.advertise("camera/right_image", 1);
    sensor_msgs::ImagePtr msg_right;
    image_transport::Publisher pub_depth = it.advertise("camera/depth_image", 1);
    sensor_msgs::ImagePtr msg_depth;

    
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

    ros::Rate loop_rate(10);

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
            cv::cvtColor(imright, imright, CV_RGBA2RGB);
            msg_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imright).toImageMsg();
            

            // get left image
            slMat2cvMat(zed->retrieveImage(SIDE::LEFT)).copyTo(imleft);
            nm_left = dleft + to_string(tstamp) + "_" + to_string(cnt) + string(".png");
            //cv::imwrite(nm_left, imleft);
            cv::cvtColor(imleft, imleft, CV_RGBA2RGB);
            msg_left = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imleft).toImageMsg();

            // get depth image
            sl::zed::Mat depth = zed->retrieveMeasure(MEASURE::DEPTH);  // get the pointer
            slMat2cvMat(zed->normalizeMeasure(MEASURE::DEPTH)).copyTo(imdepth);
            nm_depth = ddepth + to_string(tstamp) + "_" + to_string(cnt) + string(".png");
            //cv::imwrite(nm_depth, imdepth);
            cv::cvtColor(imdepth, imdepth, CV_RGBA2RGB);
            msg_depth = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imdepth).toImageMsg();

            // publish images
            pub_left.publish(msg_left);
            pub_right.publish(msg_right);
            pub_depth.publish(msg_depth);

            cnt++;

        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return 0;
}
