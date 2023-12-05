#include<iostream>
#include<librealsense2/rs.hpp>
#include<opencv2/opencv.hpp>
#include<fmt/core.h>
using namespace  cv;
using namespace rs2;
float get_depth_scale(rs2::device dev)
{
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}
void detectdistance(Mat &depth_imgae,Size range,rs2::pipeline_profile profile){
    float scale=get_depth_scale(profile.get_device());
    // float scale=get_depth_scale(profile.get_device() );
    Point center(depth_imgae.cols/2,depth_imgae.rows/2);
    Rect  Rectrange(center.x-range.width,center.y-range.height/2,range.width,range.height);
    int executive_point=0;
    float sum_distance=0;
    for(int i=Rectrange.x;i<Rectrange.x+range.width;i++){
        for(int j=Rectrange.y;j<Rectrange.y+range.height;j++){
            if(depth_imgae.at<uint16_t>(i,j)){
                sum_distance+=depth_imgae.at<uint16_t>(i,j)*scale;
                executive_point++;
            }
        }
    }
    float average_distance=sum_distance/executive_point;
    fmt::print("sum_distance=={}\nexecutive_point=={}\naverage_distance=={}\n",
                sum_distance,
                executive_point,
                average_distance);
}

int main(){
    namedWindow("展示深度彩色图像", WINDOW_AUTOSIZE);
    namedWindow("展示彩色图像",WINDOW_AUTOSIZE);
    namedWindow("展示深度图像",WINDOW_AUTOSIZE);
    rs2::colorizer color_map;
    rs2::pipeline p;
    rs2::config  pipe_config;
    pipe_config.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_Z16,30);
    pipe_config.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_BGR8,30);
    rs2::pipeline_profile profile =p.start(pipe_config);
    while(1){
        rs2::frameset data=p.wait_for_frames();//等待下一帧
        //对齐
        rs2::frame depth_frames = data.get_depth_frame().apply_filter(color_map);// RGB框架    
        rs2::frame color_frame=data.get_color_frame();
        rs2::frame depth_frames_a=data.get_depth_frame();
        int d_w=depth_frames.as<rs2::video_frame>().get_width();
        int d_h=depth_frames.as<rs2::video_frame>().get_height();
        int c_w=color_frame.as<rs2::video_frame>().get_width();
        int c_h=color_frame.as<rs2::video_frame>().get_height();
        Mat color_image(Size(c_w,c_h),CV_8UC3,(void*)color_frame.get_data(), Mat::AUTO_STEP);
        Mat depth_color_image(Size(d_w,d_h),CV_8UC3,(void*)depth_frames.get_data(), Mat::AUTO_STEP);
        Mat depth_color_image_a(Size(d_w,d_h),CV_16UC1,(void*)depth_frames_a.get_data(), Mat::AUTO_STEP);
        // fmt::print("c_w={} c_h={} d_w={} d_h={}\n",c_w,c_h,d_w,d_h);
        cv::cvtColor(depth_color_image, depth_color_image, cv::COLOR_RGB2BGR);
        //计算摄像头到区域的平均距离
        detectdistance(depth_color_image_a,Size(20,20),profile);
        imshow("展示深度彩色图像",depth_color_image);
        imshow("展示彩色图像",color_image);    rs2::device dev;
        imshow("展示深度图像",depth_color_image_a);
        waitKey(1);
    }
    return 0;
}