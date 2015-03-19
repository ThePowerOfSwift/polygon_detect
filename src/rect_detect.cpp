#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::Publisher rect_pub;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    rect_pub = nh_.advertise<geometry_msgs::Quaternion>("rectangle_ends", 1);
    //rect_pub = nh_.advertise<geometry_msgs::Twist>("rectangle_ends", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    //cv::Mat img_bgr;
    //cv::Mat img_hsv;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      /*img_bgr = cv_ptr->image;
      cv::cvtColor(img_bgr , img_hsv , CV_BGR2HSV);	//convert rgb to hsv
      for(int i=0;i<cv_ptr->image.rows;i++)
      {
      	for(int j=0;j<cv_ptr->image.cols;j++)
      	{
      		cv_ptr->image.at<cv::Vec3b>(i,j)[0] = img_hsv.at<cv::Vec3b>(i,j)[0];
      		cv_ptr->image.at<cv::Vec3b>(i,j)[1] = img_hsv.at<cv::Vec3b>(i,j)[1];
      		cv_ptr->image.at<cv::Vec3b>(i,j)[2] = img_hsv.at<cv::Vec3b>(i,j)[2];
      	}
      }*/
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //smoothing the image using Gaussian filter
    double sig = 50.0;
    for(int i=5;i<cv_ptr->image.rows-5;i++)
    {
    	for(int j=5;j<cv_ptr->image.cols-5;j++)
    	{
    		cv::Vec3b pixel;
    		double val1=0.0, val2=0.0, val3=0.0;
    		double norm=0;
    		for(int k=-5;k<=5;k++)
    		{
    			double val = (1/(2*3.14*sig*sig)) * exp((k*k + k*k)*(-0.5/(sig*sig)));
    			norm += 2*val;
    			val1 += val*cv_ptr->image.at<cv::Vec3b>(i+k,j)[0] + val*cv_ptr->image.at<cv::Vec3b>(i,j+k)[0];
    			val2 += val*cv_ptr->image.at<cv::Vec3b>(i+k,j)[1] + val*cv_ptr->image.at<cv::Vec3b>(i,j+k)[1];
    			val3 += val*cv_ptr->image.at<cv::Vec3b>(i+k,j)[2] + val*cv_ptr->image.at<cv::Vec3b>(i,j+k)[2];
    		}
    		val1 /= norm; val2 /= norm; val3 /= norm;
    		pixel[0]=(int)val1; pixel[1]=(int)val2;pixel[2]=(int)val3;
    		cv_ptr->image.at<cv::Vec3b>(i,j) = pixel;
    	}
    }

    

    int arr[cv_ptr->image.rows][cv_ptr->image.cols];
    float row_upend=0.0, row_downend=0.0, col_leftend=0.0, col_rightend=0.0;
    int col_freq_left[cv_ptr->image.cols];
    int col_freq_right[cv_ptr->image.cols];
    int row_freq_up[cv_ptr->image.rows];
    int row_freq_down[cv_ptr->image.rows];

    //initializing the arrays
    for(int i=0;i<cv_ptr->image.rows;i++)
    {
      row_freq_up[i]=0; row_freq_down[i]=0;
    }
    for(int i=0;i<cv_ptr->image.cols;i++)
    {
      col_freq_left[i]=0; col_freq_right[i]=0;
    }

    /*int red_thresh = 170;
    int green_thresh = 180;
    int blue_thresh = 205;*/
    int red_thresh = 120;
    int green_thresh = 90;
    int blue_thresh = 90;

    //traversing left to right
    for(int i=0; i<cv_ptr->image.rows; i++)
    {
        for(int j=1; j<cv_ptr->image.cols-1; j++)
        {

          cv::Vec3b pixel = cv_ptr->image.at<cv::Vec3b>(i,j);
          cv::Vec3b lpixel = cv_ptr->image.at<cv::Vec3b>(i,j-1);
          cv::Vec3b rpixel = cv_ptr->image.at<cv::Vec3b>(i,j+1);

          bool current = pixel[2]>red_thresh && pixel[0]<blue_thresh && pixel[1]<green_thresh;
          bool left = lpixel[2]>red_thresh && lpixel[0]<blue_thresh && lpixel[1]<green_thresh;
          bool right = rpixel[2]>red_thresh && rpixel[0]<blue_thresh && rpixel[1]<green_thresh;


          //turning filtered red to white and rest black
          if(pixel[2]>red_thresh && pixel[0]<blue_thresh && pixel[1]<green_thresh)
          {
            pixel[0]=0;pixel[1]=0;pixel[2]=255;
          }
          else
          {
            pixel[0]=0;pixel[1]=0;pixel[2]=0;
          }
          cv_ptr->image.at<cv::Vec3b>(i,j) = pixel;



          if(current && !left)
          {
            col_freq_left[j]++;
          }
          if(current && !right)
          {
            col_freq_right[j]++;
          }
        }
    }

    //traversing top to bottom
    for(int j=0; j<cv_ptr->image.cols; j++)
    {
        for(int i=1; i<cv_ptr->image.rows-1; i++)
        {

          cv::Vec3b pixel = cv_ptr->image.at<cv::Vec3b>(i,j);
          cv::Vec3b upixel = cv_ptr->image.at<cv::Vec3b>(i-1,j);
          cv::Vec3b dpixel = cv_ptr->image.at<cv::Vec3b>(i+1,j);

          bool current = pixel[2]>red_thresh && pixel[0]<blue_thresh && pixel[1]<green_thresh;
          bool up = upixel[2]>red_thresh && upixel[0]<blue_thresh && upixel[1]<green_thresh;
          bool down = dpixel[2]>red_thresh && dpixel[0]<blue_thresh && dpixel[1]<green_thresh;
          
          if(current && !up)
          {
            row_freq_up[i]++;
          }
          if(current && !down)
          {
            row_freq_down[i]++;
          }
        }
    }

    int lsumfreq=0, rsumfreq=0;
    for(int i=0;i<cv_ptr->image.cols;i++) //weighted mean for column
    {
      lsumfreq += col_freq_left[i];
      rsumfreq += col_freq_right[i];

      col_leftend += i*col_freq_left[i];
      col_rightend += i*col_freq_right[i];
    }
    col_leftend /= lsumfreq;
    col_rightend /= rsumfreq;

    int usumfreq=0, dsumfreq=0;
    for(int i=0;i<cv_ptr->image.rows;i++) //weighted mean for row
    {
      usumfreq += row_freq_up[i];
      dsumfreq += row_freq_down[i];

      row_upend += i*row_freq_up[i];
      row_downend += i*row_freq_down[i];
    }
    row_upend /= usumfreq;
    row_downend /= dsumfreq;

    //publishing the end rows and columns of red rectangle

    geometry_msgs::Quaternion quat;
    quat.x = 0; quat.y = 0; quat.z = 0; quat.w = 0;
    quat.x = col_leftend;
    quat.y = col_rightend;
    quat.z = row_upend;
    quat.w = row_downend;

    rect_pub.publish(quat);

    /*geometry_msgs::Twist twist;
    twist.linear.x = col_leftend;
    twist.linear.y = col_rightend;
    twist.angular.x = row_upend;
    twist.angular.y = row_downend;

    rect_pub.publish(twist);*/

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
