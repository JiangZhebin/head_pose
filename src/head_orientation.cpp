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



#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>
#include <iostream>




class ImageConverter
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub_rgb;
  image_transport::Subscriber image_sub_depth;
  image_transport::Publisher  image_pub_;

 // dlib::image_window win;

public:
  ImageConverter()
    :it(nh)
  {

  image_pub_ = it.advertise("/image_converter/output_video", 1 );

  image_sub_rgb = it.subscribe("/camera/rgb/image_rect_color", 1, &ImageConverter::imageCb, this);
  ROS_INFO("subscribe topic /camera/rgb/image_rect_color");

  ros::spin();
  }



  void display_face_features(cv::Mat &image,const std::vector<dlib::full_object_detection>& dets, const cv::Scalar color = cv::Scalar(0,255,0))
  {

    std::vector<cv::LineTypes> lines;

    for (unsigned long i=0 ; i < dets.size(); ++i)
    {
      ROS_INFO("enter display_face_features");
      std::vector<cv::Point2d> cvpart;
      const dlib::full_object_detection& d = dets[i];
      ROS_INFO("before converting points");
      //TODO!!! DEBUG
      for(unsigned long j=0 ; j < 68;++j){
        cvpart.push_back( cv::Point2l(d.part(j).x(),d.part(j).y()));
      }
      ROS_INFO("before drawing");
      // Around Chin. Ear to Ear
      for (unsigned long i = 1; i <= 16; ++i)
          cv::line(image,cvpart[i],cvpart[i-1], color);

      // Line on top of nose
      for (unsigned long i = 28; i <= 30; ++i)
          cv::line(image,cvpart[i],cvpart[i-1], color);

      // left eyebrow
      for (unsigned long i = 18; i <= 21; ++i)
          cv::line(image,cvpart[i],cvpart[i-1], color);
      // Right eyebrow
      for (unsigned long i = 23; i <= 26; ++i)
          cv::line(image,cvpart[i],cvpart[i-1], color);
      // Bottom part of the nose
      for (unsigned long i = 31; i <= 35; ++i)
          cv::line(image,cvpart[i],cvpart[i-1], color);
      // Line from the nose to the bottom part above
      cv::line(image, cvpart[30],cvpart[35],color);

      // Left eye
      for (unsigned long i = 37; i <= 41; ++i)
          cv::line(image,cvpart[i],cvpart[i-1], color);
      cv::line(image, cvpart[36],cvpart[41],color);

      // Right eye
      for (unsigned long i = 43; i <= 47; ++i)
          cv::line(image,cvpart[i],cvpart[i-1], color);
      cv::line(image, cvpart[42],cvpart[47],color);

      // Lips outer part
      for (unsigned long i = 49; i <= 59; ++i)
          cv::line(image,cvpart[i],cvpart[i-1], color);
      cv::line(image, cvpart[48],cvpart[59],color);

      // Lips inside part
      for (unsigned long i = 61; i <= 67; ++i)
          cv::line(image,cvpart[i],cvpart[i-1], color);
      cv::line(image, cvpart[60],cvpart[67],color);

    }

  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    cv::Mat image;
    cv_bridge::CvImagePtr cv_ptr;

    cv::Mat shapes_Mat;
    std::vector<cv::Mat> ccimage;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    image = cv_ptr->image;

    std::string shape_predictor_name = "/usr/local/share/attention_tracker/shape_predictor_68_face_landmarks.dat";
    dlib::frontal_face_detector detector=dlib::get_frontal_face_detector();
    dlib::shape_predictor shape_predictor;
    dlib::deserialize(shape_predictor_name) >> shape_predictor;

    dlib::cv_image<dlib::bgr_pixel> cimg(image);
    std::vector<dlib::rectangle> faces = detector(cimg);
    std::vector<dlib::full_object_detection> shapes;

    for (unsigned long i=0;i<faces.size();++i) {
      shapes.push_back(shape_predictor(cimg,faces[i]));

    cv::rectangle(image, cv::Point2i(faces[i].left(),faces[i].top()),cv::Point2i(faces[i].right()+1,faces[i].bottom()+1),cv::Scalar(255,0,0));
    }

    display_face_features(image,shapes);

  // shapes_Mat=dlib::toMat(shapes);

/*
    for(unsigned long i=0; i<shapes.size();++i){

      cv::Point2d Point1 = cv::Point2d(dlib::render_face_detections(shapes)[i].p1.x(),dlib::render_face_detections(shapes)[i].p1.y());
      cv::Point2d Point2 = cv::Point2d(dlib::render_face_detections(shapes)[i].p1.x(),dlib::render_face_detections(shapes)[i].p1.y());
      cv::line(image,Point1,Point2,cv::Scalar(255,0,0),5);
   }

*/
    image_pub_.publish(cv_ptr->toImageMsg());


  //  win.set_image(cimg);
  //  win.add_overlay(dlib::render_face_detections(shapes));



    ROS_INFO("detected faces published");

  }
} ;

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"face_detection");

  ImageConverter ic;












}
