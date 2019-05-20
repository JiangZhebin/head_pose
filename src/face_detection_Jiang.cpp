#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>


#include "opencv/cxcore.hpp"
#include "opencv/cv.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>
#include <stdio.h>






class FaceDetector
{

  cv::String face_cascade_name = "/home/zhebin/opencv/data/haarcascades/haarcascade_frontalface_alt.xml";
  cv::String eyes_cascade_name = "/home/zhebin/opencv/data/haarcascades/haarcascade_eye_tree_eyeglasses.xml";
  cv::CascadeClassifier face_cascade;
  cv::CascadeClassifier eyes_cascade;

  //Parameters
public:
  FaceDetector(cv::Mat &frame)
  {
    if(!face_cascade.load(face_cascade_name))
    {
      printf("--(!)Error loading\n");
    }
    if(!eyes_cascade.load(eyes_cascade_name)){
      printf("--(!)Error loading\n");
    }

    std::vector<cv::Rect> faces;
    cv::Mat frame_gray;
    cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
    cv::equalizeHist(frame_gray, frame_gray);
    cv::imshow("face_detection",frame_gray);
    face_cascade.detectMultiScale(frame_gray,faces);

    for(size_t i=0; i<faces.size();i++){

      cv::Point face_upper_left(faces[i].x, faces[i].y );
      cv::Point face_down_right(faces[i].x+faces[i].width, faces[i].y + faces[i].height);
      cv::Scalar face_color(255,0,0);
      cv::rectangle(frame,face_upper_left,face_down_right,face_color,4);
      ROS_INFO("faces extracted");
      cv::Mat faceROI = frame_gray(faces[i]); // extract face pixels from the whole image
      std::vector<cv::Rect> eyes;

      eyes_cascade.detectMultiScale(faceROI, eyes);
      for(size_t j=0; j<eyes.size(); j++){
        cv::Point eye_upper_left(faces[i].x+eyes[j].x, faces[i].y + eyes[j].y);
        cv::Point eye_down_right(faces[i].x + eyes[j].x + eyes[j].width , faces[i].y + eyes[j].y + eyes[j].height);
        cv::Scalar eye_color(0,255,0);
        cv::rectangle(frame, eye_upper_left, eye_down_right, eye_color,4);
        ROS_INFO("eyes extracted");
      }

    }


  }
  ~FaceDetector()
  {

  }


};

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_rgb;
  image_transport::Subscriber image_sub_depth;
  image_transport::Publisher  image_pub_;

public:
  cv::Mat image;
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("/image_converter/output_video", 1 );
    image_sub_rgb = it_.subscribe("/camera/rgb/image_rect_color", 1, &ImageConverter::imageCb, this);
    image_sub_depth=it_.subscribe("/camera/depth/image",1,&ImageConverter::depth_imageCb,this);
    ROS_INFO("subscribe topic /camera/rgb/image_rect_color");

  }

  ~ImageConverter()
  {

  }
  void depth_imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_depth_ptr;
    try{
      cv_depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1);
    } catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    FaceDetector fd(cv_ptr->image);
    image_pub_.publish(cv_ptr->toImageMsg());
    ROS_INFO("detected faces published");

  }

};


int main(int argc, char **argv)
{

  ros::init(argc, argv, "face_detection");
  ImageConverter ic;
  ros::spin();
  ROS_INFO("done");
  return 0;

}
