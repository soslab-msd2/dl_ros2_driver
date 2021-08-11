
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"

#include <opencv2/opencv.hpp>


#include "dl_driver.h"


using namespace std::chrono_literals;


class DLROS2Driver : public rclcpp::Node
{
public:
    DLROS2Driver() : Node("dl_ros2_driver_node")
    {
        DeclareParam();
        GetParam();
        
        distance_image_pub = create_publisher<sensor_msgs::msg::Image>(pub_topicname_distance_image, 10);
        amplitude_image_pub = create_publisher<sensor_msgs::msg::Image>(pub_topicname_amplitude_image, 10);
        pointcloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>(pub_topicname_pointcloud, 10);

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        timer = create_wall_timer(20ms, std::bind(&DLROS2Driver::TimerCallback, this));

        InitDL();
    }

    ~DLROS2Driver()
    {
        dl->StopStream();

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        delete dl;
    }


private:
    /************************** Functions *****************************/
    void DeclareParam(void);
    void GetParam(void);

    void TimerCallback(void);

    void InitDL(void);
    void CvtToMatDistance(cv::Mat image, const std::vector<uint16_t>& data);
    void CvtToMatAmplitude(cv::Mat image, const std::vector<uint16_t>& data);
    void CvtToMatRange(cv::Mat image, const std::vector<uint16_t>& data, const std::vector<uint16_t>& data_amplitude);
    void CvtToPointcloudMsg(sensor_msgs::msg::PointCloud2& pointcloud2_msg, cv::Mat image);
    std::string mat_type2encoding(int mat_type);
    void PubImage(const cv::Mat& image_distance, const cv::Mat& image_amplitude, const sensor_msgs::msg::PointCloud2& pointcloud2_msg);


private:
    /************************** Launch variables *****************************/
    std::string dl_ip = "10.10.31.180";
    int dl_tcp_port = 50660;
    int pc_port = 45454;

    int max_distance = 2000;
    int integration_time_low = 10;
    int integration_time_mid = 100;
    int integration_time_high = 1000;
    int integration_time_grayscale = 1000;
    int hdr = 0;
    int modulation_frequency = 0;
    int modulation_channel = 0;
    int modulation_auto_channel = 0;
    int min_amplitude = 0;
    int offset = 0;

    double cam_cal_fx = 204.374692;
    double cam_cal_fy = 206.172834;
    double cam_cal_cx = 164.42;
    double cam_cal_cy = 122.842145;
    double cam_cal_k1 = -0.378984;
    double cam_cal_k2 = 0.108586;
    double cam_cal_p1 = -0.001025;
    double cam_cal_p2 = -0.001098;

    bool filter_on = true;
    bool flip_rightleft = false;
    bool flip_updown = false;

    std::string frame_id = "base_link";
    std::string pub_topicname_distance_image = "image_distance";
    std::string pub_topicname_amplitude_image = "image_amplitude";
    std::string pub_topicname_pointcloud = "pointcloud";

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr distance_image_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr amplitude_image_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;


private:
    /************************** Other variables *****************************/
    DL* dl;
    int last_image_num = 0;
    
};

void DLROS2Driver::DeclareParam(void)
{
    declare_parameter("dl_ip", dl_ip);
    declare_parameter("dl_tcp_port", dl_tcp_port);
    declare_parameter("pc_port", pc_port);

    declare_parameter("max_distance", max_distance);
    declare_parameter("integration_time_low", integration_time_low);
    declare_parameter("integration_time_mid", integration_time_mid);
    declare_parameter("integration_time_high", integration_time_high);
    declare_parameter("integration_time_grayscale", integration_time_grayscale);
    declare_parameter("hdr", hdr);
    declare_parameter("modulation_frequency", modulation_frequency);
    declare_parameter("modulation_channel", modulation_channel);
    declare_parameter("modulation_auto_channel", modulation_auto_channel);
    declare_parameter("min_amplitude", min_amplitude);
    declare_parameter("offset", offset);

    declare_parameter("cam_cal_fx", cam_cal_fx);
    declare_parameter("cam_cal_fy", cam_cal_fy);
    declare_parameter("cam_cal_cx", cam_cal_cx);
    declare_parameter("cam_cal_cy", cam_cal_cy);
    declare_parameter("cam_cal_k1", cam_cal_k1);
    declare_parameter("cam_cal_k2", cam_cal_k2);
    declare_parameter("cam_cal_p1", cam_cal_p1);
    declare_parameter("cam_cal_p2", cam_cal_p2);
    
    declare_parameter("filter_on", filter_on);
    declare_parameter("flip_rightleft", flip_rightleft);
    declare_parameter("flip_updown", flip_updown);

    declare_parameter("frame_id", frame_id);
    declare_parameter("pub_topicname_distance_image", pub_topicname_distance_image);
    declare_parameter("pub_topicname_amplitude_image", pub_topicname_amplitude_image);
    declare_parameter("pub_topicname_pointcloud", pub_topicname_pointcloud);
}

void DLROS2Driver::GetParam(void)
{
    dl_ip = get_parameter("dl_ip").as_string();
    dl_tcp_port = get_parameter("dl_tcp_port").as_int();
    pc_port = get_parameter("pc_port").as_int();

    max_distance = get_parameter("max_distance").as_int();
    integration_time_low = get_parameter("integration_time_low").as_int();
    integration_time_mid = get_parameter("integration_time_mid").as_int();
    integration_time_high = get_parameter("integration_time_high").as_int();
    integration_time_grayscale = get_parameter("integration_time_grayscale").as_int();
    hdr = get_parameter("hdr").as_int();
    modulation_frequency = get_parameter("modulation_frequency").as_int();
    modulation_channel = get_parameter("modulation_channel").as_int();
    modulation_auto_channel = get_parameter("modulation_auto_channel").as_int();
    min_amplitude = get_parameter("min_amplitude").as_int();
    offset = get_parameter("offset").as_int();

    cam_cal_fx = get_parameter("cam_cal_fx").as_double();
    cam_cal_fy = get_parameter("cam_cal_fy").as_double();
    cam_cal_cx = get_parameter("cam_cal_cx").as_double();
    cam_cal_cy = get_parameter("cam_cal_cy").as_double();
    cam_cal_k1 = get_parameter("cam_cal_k1").as_double();
    cam_cal_k2 = get_parameter("cam_cal_k2").as_double();
    cam_cal_p1 = get_parameter("cam_cal_p1").as_double();
    cam_cal_p2 = get_parameter("cam_cal_p2").as_double();

    filter_on = get_parameter("filter_on").as_bool();
    flip_rightleft = get_parameter("flip_rightleft").as_bool();
    flip_updown = get_parameter("flip_updown").as_bool();

    frame_id = get_parameter("frame_id").as_string();
    pub_topicname_distance_image = get_parameter("pub_topicname_distance_image").as_string();
    pub_topicname_amplitude_image = get_parameter("pub_topicname_amplitude_image").as_string();
    pub_topicname_pointcloud = get_parameter("pub_topicname_pointcloud").as_string();
}

void DLROS2Driver::TimerCallback(void)
{
    DL::distance_amplitude_img_t distance_amplitude_img;
    dl->GetDistanceAmplitudeImage(distance_amplitude_img);

    if(last_image_num!=distance_amplitude_img.payload_header.img_num)
    {
        last_image_num = distance_amplitude_img.payload_header.img_num;

        cv::Mat image_distance = cv::Mat::zeros(distance_amplitude_img.payload_header.height, distance_amplitude_img.payload_header.width, CV_8UC3);
        cv::Mat image_amplitude = cv::Mat::zeros(distance_amplitude_img.payload_header.height, distance_amplitude_img.payload_header.width, CV_8UC3);
        cv::Mat image_range = cv::Mat::zeros(distance_amplitude_img.payload_header.height, distance_amplitude_img.payload_header.width, CV_64FC2);

        CvtToMatDistance(image_distance, distance_amplitude_img.distance);
        CvtToMatAmplitude(image_amplitude, distance_amplitude_img.amplitude);
        CvtToMatRange(image_range, distance_amplitude_img.distance, distance_amplitude_img.amplitude);

        cv::Mat image_amplitude_gray;
        cv::cvtColor(image_amplitude, image_amplitude_gray, cv::COLOR_BGR2GRAY);

        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
        clahe->setClipLimit(3.0);
        clahe->setTilesGridSize(cv::Size(3,3));
        clahe->apply(image_amplitude_gray, image_amplitude_gray);

        sensor_msgs::msg::PointCloud2 pointcloud2_msg;
        CvtToPointcloudMsg(pointcloud2_msg, image_range);

        PubImage(image_distance, image_amplitude_gray, pointcloud2_msg);
    }
}

void DLROS2Driver::InitDL(void)
{
    std::cout << "\n init DL\n" << std::endl;

    dl = new DL(dl_ip, dl_tcp_port, pc_port);
    dl->SetIntegrationTimes(integration_time_low, integration_time_mid, integration_time_high, integration_time_grayscale);
    // dl->SetHDR(hdr);
    dl->SetModulation(modulation_frequency, modulation_channel, modulation_auto_channel);
    dl->SetMinAmplitude(min_amplitude);
    dl->SetOffset(offset);
    dl->SetStreamDistanceAmplitude();
}

void DLROS2Driver::CvtToMatDistance(cv::Mat image_out, const std::vector<uint16_t>& data)
{
    cv::Mat image = image_out.clone();

    for(int i=0; i<data.size(); i++)
    {
        double normalized_radius = ((double)data[i])/max_distance*4.0;
        double height_sublevel = normalized_radius - (int)normalized_radius;

        int x = i % image.cols;
        int y = i / image.cols;

        if( x>=0 && x<image.cols && y>=0 && y<image.rows )
        {
            if(data[i]<=(max_distance))
            {
                if(0.0<=normalized_radius && normalized_radius<1.0)
                    image.at<cv::Vec3b>(y,x) = { 0, (int)(height_sublevel*255), 255 };
                else if(1.0<=normalized_radius && normalized_radius<2.0) 
                    image.at<cv::Vec3b>(y,x) = { 0, 255, (int)((1 - height_sublevel)*255) };
                else if(2.0<=normalized_radius && normalized_radius<3.0) 
                    image.at<cv::Vec3b>(y,x) = { (int)(height_sublevel*255), 255, 0 };
                else if(3.0<=normalized_radius && normalized_radius<4.0) 
                    image.at<cv::Vec3b>(y,x) = { 255, (int)((1 - height_sublevel)*255), 0 };
                else if(4.0==normalized_radius) 
                    image.at<cv::Vec3b>(y,x) = { 255, 0, 0 };
            }
            else if(data[i]==64002)
            {
                image.at<cv::Vec3b>(y,x) = { 255, 255, 255 };
            }
        }
    }
    
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat distortion_coefficients = cv::Mat::zeros(1, 5, CV_64FC1);
    camera_matrix = (cv::Mat1d(3, 3) << cam_cal_fx, 0.0, cam_cal_cx, 0.0, cam_cal_fy, cam_cal_cy, 0.0, 0.0, 1.0 ); 
    distortion_coefficients = (cv::Mat1d(1, 5) << cam_cal_k1, cam_cal_k2, cam_cal_p1, cam_cal_p2, 0.0);
    undistort(image, image_out, camera_matrix, distortion_coefficients);
}

void DLROS2Driver::CvtToMatAmplitude(cv::Mat image_out, const std::vector<uint16_t>& data)
{
    cv::Mat image = image_out.clone();

    double max_value = 1500;//-99999;
    double min_value = 10;//99999;
    // for(int i=0; i<data.size(); i++)
    // {
    //     if((double)data[i]>max_value && (double)data[i]<6000) max_value = (double)data[i];
    //     if((double)data[i]<min_value && (double)data[i]>0) min_value = (double)data[i];
    // }
    
    for(int i=0; i<data.size(); i++)
    {
        double normalized_radius = ((double)data[i]-min_value)/(max_value-min_value)*3.0;
        double height_sublevel = normalized_radius - (int)normalized_radius;

        int x = i % image.cols;
        int y = i / image.cols;

        if( x>=0 && x<image.cols && y>=0 && y<image.rows )
        {
            if(data[i]<=(max_value+min_value))
            {
                if(0.0<=normalized_radius && normalized_radius<=1.0)
                    image.at<cv::Vec3b>(y,x) = { 0, (int)(height_sublevel*255), 0 };
                else if(1.0<normalized_radius && normalized_radius<=2.0) 
                    image.at<cv::Vec3b>(y,x) = { 0, 255, (int)((height_sublevel)*255) };
                else if(2.0<normalized_radius && normalized_radius<=3.0) 
                    image.at<cv::Vec3b>(y,x) = { (int)(height_sublevel*255), 255, 255 };
                else if(3.0<normalized_radius) 
                    image.at<cv::Vec3b>(y,x) = { 255, 255, 255 };
            }
            else if(data[i]==64002)
            {
                image.at<cv::Vec3b>(y,x) = { 255, 255, 255 };
            }
        }
    }

    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat distortion_coefficients = cv::Mat::zeros(1, 5, CV_64FC1);
    camera_matrix = (cv::Mat1d(3, 3) << cam_cal_fx, 0.0, cam_cal_cx, 0.0, cam_cal_fy, cam_cal_cy, 0.0, 0.0, 1.0 ); 
    distortion_coefficients = (cv::Mat1d(1, 5) << cam_cal_k1, cam_cal_k2, cam_cal_p1, cam_cal_p2, 0.0);
    undistort(image, image_out, camera_matrix, distortion_coefficients);
}

void DLROS2Driver::CvtToMatRange(cv::Mat image_out, const std::vector<uint16_t>& data, const std::vector<uint16_t>& data_amplitude)
{
    cv::Mat image = image_out.clone();

    for(int i=0; i<data.size(); i++)
    {
        int x = i % image.cols;
        int y = i / image.cols;

        if((double)data[i]>0 && (double)data[i]<20000)
        {
            image.at<cv::Vec2d>(y,x)[0] = (double)data[i]/1000.0;
        }
        else
        {
            image.at<cv::Vec2d>(y,x)[0] = -10000000.0;
        }

        image.at<cv::Vec2d>(y,x)[1] = (double)data_amplitude[i];
    }
    
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat distortion_coefficients = cv::Mat::zeros(1, 5, CV_64FC1);
    camera_matrix = (cv::Mat1d(3, 3) << cam_cal_fx, 0.0, cam_cal_cx, 0.0, cam_cal_fy, cam_cal_cy, 0.0, 0.0, 1.0 ); 
    distortion_coefficients = (cv::Mat1d(1, 5) << cam_cal_k1, cam_cal_k2, cam_cal_p1, cam_cal_p2, 0.0);
    undistort(image, image_out, camera_matrix, distortion_coefficients);
}

void DLROS2Driver::CvtToPointcloudMsg(sensor_msgs::msg::PointCloud2& pointcloud2_msg, cv::Mat image)
{
    sensor_msgs::msg::PointCloud pointcloud_msg;
    pointcloud_msg.header.frame_id = frame_id;
    pointcloud_msg.channels.resize(1);
    pointcloud_msg.channels[0].name = "intensities";

    if(filter_on==true)
    {
        for(int x_idx=0; x_idx<image.cols; x_idx++)    
        {
            for(int y_idx=0; y_idx<(image.rows-1); y_idx++)
            {
                if(image.at<cv::Vec2d>(y_idx,x_idx)[0]>0 && image.at<cv::Vec2d>(y_idx+1,x_idx)[0]>0)
                {
                    double diff = (image.at<cv::Vec2d>(y_idx,x_idx)[0]-image.at<cv::Vec2d>(y_idx+1,x_idx)[0])/2.0;
                    if(diff>0.025*image.at<cv::Vec2d>(y_idx,x_idx)[0]) image.at<cv::Vec2d>(y_idx,x_idx)[0] = 0.0;
                    if(diff<-0.025*image.at<cv::Vec2d>(y_idx,x_idx)[0]) image.at<cv::Vec2d>(y_idx,x_idx)[0] = 0.0;
                }
            }
        }

        for(int y_idx=0; y_idx<image.rows; y_idx++)    
        {
            for(int x_idx=0; x_idx<(image.cols-1); x_idx++)
            {
                if(image.at<cv::Vec2d>(y_idx,x_idx)[0]>0 && image.at<cv::Vec2d>(y_idx,x_idx+1)[0]>0)
                {
                    double diff = (image.at<cv::Vec2d>(y_idx,x_idx)[0]-image.at<cv::Vec2d>(y_idx,x_idx+1)[0])/2.0;
                    if(diff>0.025*image.at<cv::Vec2d>(y_idx,x_idx)[0]) image.at<cv::Vec2d>(y_idx,x_idx)[0] = 0.0;
                    if(diff<-0.025*image.at<cv::Vec2d>(y_idx,x_idx)[0]) image.at<cv::Vec2d>(y_idx,x_idx)[0] = 0.0;
                }
            }
        }
    }
        
    for(int x_idx=0; x_idx<image.cols; x_idx++)
    {
        for(int y_idx=0; y_idx<image.rows; y_idx++)
        {
            double distance = image.at<cv::Vec2d>(y_idx,x_idx)[0] - 0.2;

            double x_n = ((double)x_idx-cam_cal_cx)/cam_cal_fx;
            double y_n = ((double)y_idx-cam_cal_cy)/cam_cal_fy;

            double point_z = distance / sqrt(1+x_n*x_n+y_n*y_n);
            double point_x = x_n*point_z;
            double point_y = y_n*point_z;
            
            geometry_msgs::msg::Point32 single_point;
            single_point.x = point_x;
            single_point.y = -point_y;
            single_point.z = point_z;

            if(distance>0.0)
            {
                pointcloud_msg.points.push_back(single_point);
                pointcloud_msg.channels[0].values.push_back((double)image.at<cv::Vec2d>(y_idx,x_idx)[1]);
            }
        }
    }

    sensor_msgs::convertPointCloudToPointCloud2(pointcloud_msg,pointcloud2_msg);
}

std::string DLROS2Driver::mat_type2encoding(int mat_type)
{
    switch (mat_type) 
    {
        case CV_8UC1:
            return "mono8";
        case CV_8UC3:
            return "bgr8";
        case CV_16SC1:
            return "mono16";
        case CV_8UC4:
            return "rgba8";
        default:
            throw std::runtime_error("Unsupported encoding type");
    }
}

void DLROS2Driver::PubImage(const cv::Mat& image_distance, const cv::Mat& image_amplitude, const sensor_msgs::msg::PointCloud2& pointcloud2_msg) 
{
    if(flip_rightleft==true)
    {
        cv::flip(image_distance, image_distance, 1);
        cv::flip(image_amplitude, image_amplitude, 1);
    } 
    if(flip_updown==true) 
    {
        cv::flip(image_distance, image_distance, 0);
        cv::flip(image_amplitude, image_amplitude, 0);
    }

    // Publish distance image
    sensor_msgs::msg::Image distance_image_msg;
    distance_image_msg.header.stamp = rclcpp::Clock().now();
    distance_image_msg.header.frame_id = frame_id;
    distance_image_msg.height = image_distance.rows;
    distance_image_msg.width = image_distance.cols;
    distance_image_msg.encoding = mat_type2encoding(image_distance.type());
    distance_image_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(image_distance.step);

    size_t size_distance = image_distance.step * image_distance.rows;
    distance_image_msg.data.resize(size_distance);
    memcpy(&distance_image_msg.data[0], image_distance.data, size_distance);

    distance_image_pub->publish(distance_image_msg);


    // Publish amplitude image
    sensor_msgs::msg::Image amplitude_image_msg;
    amplitude_image_msg.header.stamp = distance_image_msg.header.stamp;
    amplitude_image_msg.header.frame_id = frame_id;
    amplitude_image_msg.height = image_amplitude.rows;
    amplitude_image_msg.width = image_amplitude.cols;
    amplitude_image_msg.encoding = mat_type2encoding(image_amplitude.type());
    amplitude_image_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(image_amplitude.step);

    size_t size_amplitude = image_amplitude.step * image_amplitude.rows;
    amplitude_image_msg.data.resize(size_amplitude);
    memcpy(&amplitude_image_msg.data[0], image_amplitude.data, size_amplitude);

    amplitude_image_pub->publish(amplitude_image_msg);


    // Publish pointcloud2
    pointcloud_pub->publish(pointcloud2_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DLROS2Driver>());
    rclcpp::shutdown();
   
    return 0;
}

