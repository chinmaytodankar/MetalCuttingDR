#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Bool.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

class RealsenseRecorder {
    private:
        image_transport::ImageTransport _it;
        image_transport::Subscriber _sub;
        ros::Subscriber _record_flag_sub;
        ros::NodeHandle _nh;
        bool _record_flag = false;
        std::string _path, _image_path;
        int _counter = 0;
        cv::VideoWriter _videoWriter;
        int _frame_height = -1;
        int _frame_width = -1;
    public:
        RealsenseRecorder(ros::NodeHandle nh) : _it(nh) {
            _nh = nh;
            _sub = _it.subscribe("/camera/color/image_raw", 1, &RealsenseRecorder::imageCallback, this);
            _record_flag_sub = _nh.subscribe("/realsense_record", 1, &RealsenseRecorder::recordCallback, this);
        }

        void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
            // ROS_INFO("Image Received!");
            cv_bridge::CvImagePtr cv_image_ptr;
            try {
                cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch(cv_bridge::Exception& e) {
                ROS_ERROR("CV_BRIDGE_EXCEPTION: %s", e.what());
                return;
            }
            cv::Mat image = cv_image_ptr->image;
            image = rotateImage(image, 90);
            
            _frame_height = image.rows;
            _frame_width = image.cols;

            if(_record_flag) {
                ROS_INFO("Recording...");
                std::string counter_string = std::to_string(_counter);
                counter_string = std::string(6 - std::min(6, int(counter_string.length())), '0') + counter_string;
                cv::imwrite(_image_path+"/"+counter_string+".png", image);
                _videoWriter.write(image);
                _counter++;
                // _record_flag = false;
                // ROS_INFO("Recording Stopped");
            }
            cv::imshow("Image", image);
            cv::waitKey(30);
        }

        cv::Mat rotateImage(cv::Mat src, double angle) {

            // get rotation matrix for rotating the image around its center in pixel coordinates
            cv::Point2f center((src.cols-1)/2.0, (src.rows-1)/2.0);
            cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
            // determine bounding rectangle, center not relevant
            cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), src.size(), angle).boundingRect2f();
            // adjust transformation matrix
            rot.at<double>(0,2) += bbox.width/2.0 - src.cols/2.0;
            rot.at<double>(1,2) += bbox.height/2.0 - src.rows/2.0;

            cv::Mat dst;
            cv::warpAffine(src, dst, rot, bbox.size());
            return dst;
        }

        void recordCallback(const std_msgs::BoolConstPtr& msg) {
            bool prev_flag = _record_flag;
            _record_flag = msg->data;
            if(_record_flag) {
                ROS_INFO("Recording Started");
                int buffer_size = 100;
                char time_buffer[buffer_size];
                std::time_t raw_time = static_cast<time_t>(ros::Time::now().sec);
                struct tm* timeinfo = localtime(&raw_time);
                std::strftime(time_buffer, buffer_size, "%m%d%Y_%H%M%S", timeinfo);
                _path = "/home/chinmay/dr_ws/src/realsense_recorder/output/"+std::string(time_buffer);
                _image_path = _path + "/images_"+time_buffer;
                boost::filesystem::path p(_image_path);
                boost::filesystem::create_directories(p);
                _videoWriter = cv::VideoWriter(_path+"/video.avi", cv::VideoWriter::fourcc('M','J','P','G'), 30, cv::Size(_frame_width, _frame_height));
                _counter = 0;
                // ROS_INFO(_time_stamp.c_str());
                
            } else {
                ROS_INFO("Recording Stopped");
                if(!_record_flag && prev_flag) {
                    _videoWriter.release();
                    ROS_INFO("Recorder Released");
                }
            }
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "realsense_recorder_node");
    ros::NodeHandle nh;
    RealsenseRecorder realsenseRecorder(nh);
    ros::spin();
    return 0;
}