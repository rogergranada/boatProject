#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <zed/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <zedpub/Img.h>

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <sys/types.h>
#include <dirent.h>

#include <bits/stdc++.h> 
#include <sys/stat.h> 
#include <sys/types.h>

using namespace sl::zed;
using namespace std;

typedef std::vector<std::string> stringvec;

int convert_to_int(string& filename){
    int id;
    try {
        id = stoi(filename);
    } catch (invalid_argument const &e) {
        cerr << "Bad input: std::invalid_argument thrown" << endl;
    } catch (out_of_range const &e) {
        cerr << "Integer overflow: std::out_of_range thrown" << endl;
    }
    return id;
}


int last_directory(const string& name){
    int max_id = 0;
    DIR* dirp = opendir(name.c_str());
    struct dirent * dp;
    while ((dp = readdir(dirp)) != NULL) {
        string fname = dp->d_name;
        if (fname != "." and fname != ".."){
            int id_dir = convert_to_int(fname);
            if (id_dir > max_id)
                max_id = id_dir;
        }
    }
    closedir(dirp);
    return max_id;
}


bool create_directory(string path){
    char char_path[path.size() + 1];
    path.copy(char_path, path.size() + 1);
    char_path[path.size()] = '\0';
    if (mkdir(char_path, 0777) == -1) {
        cerr << "Error:  " << strerror(errno) << endl;
        return false;
    } else {
        return true;
    } 
}


int main(int argc, char** argv) {
    // infos of images
    int tstamp = time(0);
    string HOME_FOLDER = "/media/ubuntu/JetsonSSD/ZED_IMAGES/";
    int id_folder = last_directory(HOME_FOLDER);
    id_folder = id_folder + 1;
    string output_folder = HOME_FOLDER + to_string(id_folder);
    bool created = create_directory(output_folder);

    if (created == true){
        //string dright = "/media/ubuntu/JetsonSSD/" + to_string(tstamp) + "/right/";
        string dright = output_folder + "/right/";
        string nm_right;
        //string dleft = "/media/ubuntu/JetsonSSD/" + to_string(tstamp) + "/left/";
        string dleft = output_folder + "/left/";
        string nm_left;
        //string ddepth = "/media/ubuntu/JetsonSSD/" + to_string(tstamp) + "/depth/";
        string ddepth = output_folder + "/depth/";
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
    /*
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
    */ 
        int ConfidenceIdx = 100;
        
        while (ros::ok()){
    /*
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
    */
                // publish the name of the image
                zedpub::Img msg;
                stringstream ss;
                ss << to_string(cnt) + string(".png");
           
                ros::Time time = ros::Time::now();
                msg.header.stamp = time;
                msg.header.frame_id = to_string(cnt);
                msg.imgname = ss.str();
                camera_pub.publish(msg);

                cnt++;

    //        }

            ros::spinOnce();
            loop_rate.sleep();
        }
    } else {
        cout << "ERROR: Could not create folder " << output_folder <<endl;
    }
    return 0;
}
