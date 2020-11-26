#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

ros::Publisher pub_img_,pub_img_info_;
sensor_msgs::CameraInfo cam_info_;
std_msgs::Header header_image_;

static const std::string base64_chars =
"ABCDEFGHIJKLMNOPQRSTUVWXYZ"
"abcdefghijklmnopqrstuvwxyz"
"0123456789+/";


static inline bool is_base64(unsigned char c) {
    return (isalnum(c) || (c == '+') || (c == '/'));
}

std::string base64_decode(std::string const& encoded_string) {
    int in_len = encoded_string.size();
    int i = 0;
    int j = 0;
    int in_ = 0;
    unsigned char char_array_4[4], char_array_3[3];
    std::string ret;

    while (in_len-- && (encoded_string[in_] != '=') && is_base64(encoded_string[in_])) {
        char_array_4[i++] = encoded_string[in_]; in_++;
        if (i == 4) {
            for (i = 0; i < 4; i++)
                char_array_4[i] = base64_chars.find(char_array_4[i]);

            char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

            for (i = 0; (i < 3); i++)
                ret += char_array_3[i];
            i = 0;
        }
    }

    if (i) {
        for (j = i; j < 4; j++)
            char_array_4[j] = 0;

        for (j = 0; j < 4; j++)
            char_array_4[j] = base64_chars.find(char_array_4[j]);

        char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
        char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
        char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

        for (j = 0; (j < i - 1); j++) ret += char_array_3[j];
    }

    return ret;
}

void imageCallback(const std_msgs::StringConstPtr& msg)
{
  try
  {
    // cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat
    // cv::imshow("view", image);
    // cv::waitKey(10);
    std::string dec_jpg =  base64_decode(msg->data);
    std::vector<uchar> data(dec_jpg.begin(), dec_jpg.end());
    cv::Mat img = cv::imdecode(cv::Mat(data), 1);
    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; // >> message to be sent

    header_image_.seq ++; // user defined counter
    header_image_.stamp = ros::Time::now(); // time
    header_image_.frame_id = "camera";
    // img_bridge = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, image);
    img_bridge = cv_bridge::CvImage(header_image_, sensor_msgs::image_encodings::BGR8, img);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
    pub_img_.publish(img_msg); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
    // cam_info_.header = msg->header;
    cam_info_.header = header_image_;
    cam_info_.height = img.rows;
    cam_info_.width = img.cols;

    pub_img_info_.publish(cam_info_);

    // cv::imshow("view", img);
    // cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert to image!");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cam_from_web");
  ros::NodeHandle nh;
  ros::Rate r(60.0);
  ros::Subscriber sub = nh.subscribe("/camera_string", 1, imageCallback);

  pub_img_ = nh.advertise<sensor_msgs::Image>("/camera/image_raw", 1);
  pub_img_info_ = nh.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 1);

  cam_info_.distortion_model = "plumb_bob";
  cam_info_.D = {0.11972113402113273, -0.2713500793919838, 0.0031028535307321015, -0.003200812987822882, 0.0};
  cam_info_.K = {500.57274860253756, 0.0, 317.57357972230164, 0.0, 502.2879389120799, 238.4359908991159, 0.0, 0.0, 1.0};
  cam_info_.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  cam_info_.P = {502.4114990234375, 0.0, 315.2371314220363, 0.0, 0.0, 509.33612060546875, 239.50701292646954, 0.0, 0.0, 0.0, 1.0, 0.0};
  cam_info_.binning_x = 0;
  cam_info_.binning_y = 0;
  cam_info_.roi.x_offset = 0;
  cam_info_.roi.y_offset = 0;
  cam_info_.roi.height = 0;
  cam_info_.roi.width = 0;

  while(ros::ok())
  {
    

    ros::spinOnce();
    r.sleep();
        
  }// end while()
  return 0;
}