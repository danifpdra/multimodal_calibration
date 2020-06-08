#include <ros/console.h>
#include <ros/ros.h>
#include <rospack/rospack.h>

#include <pcl/common/time.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

/* openCv*/
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <stdio.h> /* printf */

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZ PointType;

class RangeImg {
public:
  // publicadores
  RangeImg();
  void GetRangeImage(cv::Mat range_img_cv);
  void loop_function();
  pcl::Grabber *interface = new pcl::OpenNIGrabber();

private:
  // rosnode
  ros::NodeHandle nh;
  ros::Publisher range_img_pub;

  image_transport::ImageTransport it;
  sensor_msgs::ImagePtr msg_range_img;
};

RangeImg::RangeImg() : it(nh) {
  range_img_pub = nh.advertise<sensor_msgs::Image>("range_img", 10);
}

void RangeImg::GetRangeImage(cv::Mat range_img_cv) {
  msg_range_img = cv_bridge::CvImage{std_msgs::Header(), "mono8", range_img_cv}
                      .toImageMsg();

  range_img_pub.publish(msg_range_img);
}

// Call back method
void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)

{
  float maxAngleWidth = (float)(57.0f * (M_PI / 180.0f));
  float maxAngleHeight = (float)(43.0f * (M_PI / 180.0f));
  float angularResolution = (float)(57.0f / 640.0f * (M_PI / 180.0f));
  Eigen::Affine3f sensorPose = Eigen::Affine3f::Identity();

  pcl::RangeImage::CoordinateFrame coordinate_frame =
      pcl::RangeImage::CAMERA_FRAME;
  float noiseLevel = 0.00;
  float minRange = 0.0f;
  int borderSize = 1;

  boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
  pcl::RangeImage &rangeImage = *range_image_ptr;

  // Range image for live stream from Kinect
  rangeImage.createFromPointCloud(*cloud, angularResolution, maxAngleWidth,
                                  maxAngleHeight, sensorPose, coordinate_frame,
                                  noiseLevel, minRange, borderSize);

  // std::cout << rangeImage.points << "\n";

  int cols = rangeImage.width;
  int rows = rangeImage.height;
  cv::Mat range_img_cv_i = cv::Mat(rows, cols, CV_64F, 0.0);

  for (size_t i = 0; i < (rangeImage.points.size()); i++) {
    float z = rangeImage.getPoint(i).z;
    // std::cout << z << " and " << i << endl;
    if (std::isnan(z) == false)
      range_img_cv_i.at<float>(i) = z;
    // std::cout << "is a number" << endl;
  }

  // std::cout << range_img_cv_i << endl;

  cv::Mat range_img_cv = cv::Mat(rows, cols, CV_8UC1, 0.0);
  cv::normalize(range_img_cv_i, range_img_cv, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  RangeImg RangeImg;
  RangeImg.GetRangeImage(range_img_cv);
}

void RangeImg::loop_function() {

  // make callback function from member function
  boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &)>
      f = boost::bind(&cloud_cb_, _1);

  // connect callback function for desired signal. In this case its a point
  // cloud with color values
  boost::signals2::connection c = interface->registerCallback(f);

  // start receiving point clouds
  interface->start();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "RangeImg");
  RangeImg reconstruct;
  // create a new grabber for OpenNI devices

  ros::Rate rate(15);
  while (ros::ok() && true) {
    reconstruct.loop_function();
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    ros::spinOnce();
    rate.sleep();
  }
  // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep
  // (1);
  // while (true)
  //   boost::this_thread::sleep(boost::posix_time::seconds(1));

  // stop the grabber
  reconstruct.interface->stop();

  return (0);
}
