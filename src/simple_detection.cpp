#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
class ObejctDetection
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Mat src_gray;
  vector<Vec3f> circles;

public:
  ObejctDetection()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
      &ObejctDetection::imageCb, this);
    image_pub_ = it_.advertise("/detection/objects", 1);
  }

  ~ObejctDetection()
  {
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::cvtColor(cv_ptr->image, src_gray, CV_RGB2GRAY); 

    // Reduce the noise so we avoid false circle detection
    cv::GaussianBlur(src_gray, src_gray, Size(9,9), 2, 2);
  
    // Apply the Hough Transform to find the circles
    cv::HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 50, 80, 0, 0);

    // Draw the circles detected
    for(size_t i = 0; i < circles.size(); i++){
  
      cv::Point  center( cvRound(circles[i][0]), cvRound(circles[i][1]));
      int radius = cvRound(circles[i][2]);

      // circle center
      cv::circle( cv_ptr->image, center, 3, Scalar(0, 255, 0), -1, 0, 0);
      cv::putText(cv_ptr->image, "Detection", center, 1, 1, Scalar(0, 0, 255), 3, 0);

      // circle outline
      cv::circle(cv_ptr->image, center, radius, Scalar(0, 0, 255), 3, 8, 0);
    }

    
    cv::Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    /// Threshold value detection boundary 
    cv::threshold( src_gray, threshold_output, 173, 255, THRESH_BINARY );
    /// Looking for contour 
    cv::findContours( threshold_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Create an inclined bounding box and ellipse for each found outline 
    vector<cv::RotatedRect> minRect( contours.size() );
    vector<cv::RotatedRect> minEllipse( contours.size() );

    for( int i = 0; i < contours.size(); i++ ){
      minRect[i] = minAreaRect( Mat(contours[i]) );
      if( contours[i].size() > 5 ){
        minEllipse[i] = fitEllipse( Mat(contours[i]) ); 
      }         
    }

    /// Draw outline and its inclined boundary box and boundary ellipse 
    cv::Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ ){
      // contour
      cv::drawContours( drawing, contours, i, Scalar(0,255,0), 1, 8, vector<Vec4i>(), 0, Point() );
      // rotated rectangle
      cv::Point2f rect_points[4]; minRect[i].points( rect_points );
      for( int j = 0; j < 4; j++ ){
        cv::line( cv_ptr->image, rect_points[j], rect_points[(j+1)%4], Scalar(0,0,255), 3, 8 );   
      }
    }

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detction");
  ObejctDetection ic;
  ros::spin();
  return 0;
}

