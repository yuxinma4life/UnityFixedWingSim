#include "ocam_ros.h"


oCam_ROS::oCam_ROS() :
  nh_(),
  nh_private_("~"),
  it_(nh_),
  info_manager_(nh_, "ocam")
{
  /*
   * Initialize ROS parameters
   */
  nh_private_.param<std::string>("device_path", device_path_, "/dev/video0");
  nh_private_.param<int>("width", width_, 640);
  nh_private_.param<int>("height", height_, 480);
  nh_private_.param<int>("fps", fps_, 80);
  nh_private_.param<std::string>("image_topic", image_topic_, "image_raw");
  nh_private_.param<std::string>("mono_image_topic", mono_image_topic_, "image_mono");
  nh_private_.param<std::string>("frame_id", frame_id_, "ocam");
  nh_private_.param<bool>("show_image", show_image_, false);
  nh_private_.param<bool>("rescale_camera_info", rescale_camera_info_, false);
  nh_private_.param<bool>("auto_exposure", auto_exposure_, true);

  /*
   * Initialize camera info manager
   */
  std::string url;
  if (nh_private_.getParam("camera_info_url", url))
  {
    if (info_manager_.validateURL(url))
    {
      info_manager_.loadCameraInfo(url);
    }
  }

  /*
   * Initialize publishers
   */
  image_pub_ = it_.advertiseCamera(image_topic_, 1);
  mono_image_pub_ = it_.advertise(mono_image_topic_, 1);

  /*
   * initialize oCam-1CGN
   *
   * [ supported image formats ]
   *
   * USB 3.0
   * 	[0] "8-bit Greyscale 1280 x 720 60 fps"
	 *	[1] "8-bit Greyscale 1280 x 960 45 fps"
	 *	[2] "8-bit Greyscale 320 x 240 160 fps"
	 * 	[3] "8-bit Greyscale 640 x 480 80 fps"
	 *
	 * USB 2.0
   * 	[0] "8-bit Greyscale 1280 x 720 30 fps"
	 *	[1] "8-bit Greyscale 1280 x 960 22.5 fps"
	 *	[2] "8-bit Greyscale 320 x 240 160 fps"
	 * 	[3] "8-bit Greyscale 640 x 480 80 fps"
	 *
   */

  /* USB 3.0 */
    /* bayer RBG 1280 x 720 60 fps */
    //camera.set_format(1280, 720, Withrobot::fourcc_to_pixformat('G','B','G','R'), 1, 60);

    /* bayer RBG 1280 x 960 45 fps */
    //camera.set_format(1280, 960, Withrobot::fourcc_to_pixformat('G','B','G','R'), 1, 45);

    /* bayer RBG 320 x 240 160 fps */
    //camera.set_format(320, 240, Withrobot::fourcc_to_pixformat('G','B','G','R'), 1, 160);

    /* bayer RBG 640 x 480 80 fps */
    //camera.set_format(640, 480, Withrobot::fourcc_to_pixformat('G','B','G','R'), 1, 80);

  /* USB 2.0 */
    /* bayer RBG 1280 x 720 30 fps */
    //camera.set_format(1280, 720, Withrobot::fourcc_to_pixformat(''G','B','G','R'), 1, 30);

    /* bayer RBG 1280 x 960 22.5 fps */
    //camera.set_format(1280, 960, Withrobot::fourcc_to_pixformat(''G','B','G','R'), 2, 45);

    /* bayer RBG 320 x 240 160 fps */
    //camera.set_format(320, 240, Withrobot::fourcc_to_pixformat(''G','B','G','R'), 1, 160);

    /* bayer RBG 640 x 480 80 fps */
    //camera.set_format(640, 480, Withrobot::fourcc_to_pixformat(''G','B','G','R'), 1, 80);

  Withrobot::Camera camera(device_path_.c_str());
  camera.set_format(width_, height_, Withrobot::fourcc_to_pixformat('G','B','G','R'), 1, fps_);

  /*
   * get current camera format (image size and frame rate)
   */
  Withrobot::camera_format camFormat;
  camera.get_current_format(camFormat);

  /*
   * Print information
   */
  std::string camName = camera.get_dev_name();
  std::string camSerialNumber = camera.get_serial_number();

  printf("dev: %s, serial number: %s\n", camName.c_str(), camSerialNumber.c_str());
  printf("----------------- Current format information ------------------\n");
  camFormat.print();
  printf("---------------------------------------------------------------\n");

  /*
   * [ supported camera controls; The double quotes are the 'get_control' and the 'set_control' function string argument values. ]
   *
   *  [0] "Gain",          Value(default [min, step, max]): 64 ( 64 [0, 1, 127] )
   *  [1] "Exposure (Absolute)", Value(default [min, step, max]): 39 ( 39 [1, 1, 625] )
   *
   */
  int brightness, exposure;
  if (!nh_private_.getParam("brightness", brightness))
    brightness = camera.get_control("Gain");
  if (!nh_private_.getParam("exposure", exposure))
    exposure = camera.get_control("Exposure (Absolute)");

  camera.set_control("Gain", brightness);
  camera.set_control("Exposure (Absolute)", exposure);

  /*
   * Start streaming
   */
  if (!camera.start()) 
  {
  	perror("Failed to start.");
  	exit(0);
  }

  /*
   * Initialize OpenCV
   */
  std::string windowName = camName + " " + camSerialNumber;
  cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
  cv::Mat colorImg(cv::Size(camFormat.width, camFormat.height), CV_8UC3);
  cv::Mat monoImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);

  /* Create OpenCV window if needed and display camera adjustment instructions*/
  if (show_image_)
  {
    // OpenCV window
    cv::namedWindow(windowName.c_str(), CV_WINDOW_KEEPRATIO|CV_WINDOW_AUTOSIZE);

    // camera adjustment instructions
    std::cout << "\n<--------- oCam Adjustment -------->\n";
    std::cout << "Quit adjustment and image view: q\n";
    std::cout << "Decrease exposure: [\n";
    std::cout << "Increase exposure: ]\n";
    std::cout << "Decrease brightness: -\n";
    std::cout << "Increase brightness: =\n";
    std::cout << "<---------------------------------->\n\n";
  }


  /*
   * Main loop
   */
  ros::Rate loop_rate(fps_);
  while (ros::ok()) 
  {
  	/* Copy a single frame(image) from camera(oCam-1MGN). This is a blocking function. */
  	int size = camera.get_frame(srcImg.data, camFormat.image_size, 1);

    /* Time for headers immediately after getting the image */
    ros::Time now = ros::Time::now();

  	/* If the error occurred, restart the camera. */
  	if (size == -1 && errno) {
	    printf("error number: %d\n", errno);
	    perror("Cannot get image from camera");
	    camera.stop();
	    camera.start();
	    continue;
  	}

	  cv::cvtColor(srcImg, colorImg, cv::COLOR_BayerGB2BGR);
    cv::cvtColor(colorImg, monoImg, cv::COLOR_BGR2GRAY);

    /* Automatically adjust exposure */
    if (auto_exposure_)
    {
      // secant method for finding exposure at target intensity
      static float intensity_min = 18;
      static float exposure_min = 1;
      static float target_intensity = 128; // 8 bit image is 0-255
      float intensity = cv::mean(monoImg).val[0]; // mean intensity
      float exposure_new = ((exposure - exposure_min)*(target_intensity - intensity_min))/(intensity - intensity_min) + exposure_min;

      // LPF the exposure to prevent large jumps inducing oscillations
      static float alpha = 0.9;
      exposure = int(alpha*exposure + (1-alpha)*exposure_new);

      // saturate exposure at its limits
      if (exposure > 625)
        exposure = 625;
      if (exposure < 1)
        exposure = 1;

      // set the new exposure
      camera.set_control("Exposure (Absolute)", exposure);
    }

    /* Build ROS image messages */
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", colorImg).toImageMsg();
    sensor_msgs::ImagePtr mono_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", monoImg).toImageMsg();

    msg->header.frame_id = frame_id_;
    mono_msg->header.frame_id = frame_id_;

    msg->header.stamp = now;
    mono_msg->header.stamp = now;

    /* Build ROS camera info message */
    info_ = info_manager_.getCameraInfo();
    if (info_.height == 0 && info_.width == 0)
    {
      info_.height = msg->height;
      info_.width = msg->width;
    }
    else if (info_.height != msg->height || info_.width != msg->width)
    {
      if (rescale_camera_info_)
      {
        int old_width = info_.width;
        int old_height = info_.height;
        rescaleCameraInfo(msg->width, msg->height);
        ROS_INFO_ONCE("Camera calibration automatically rescaled from %dx%d to %dx%d",
                      old_width, old_height, msg->width, msg->height);
      }
      else
      {
        ROS_WARN_ONCE("Calibration resolution %dx%d does not match camera resolution %dx%d. "
                      "Use rescale_camera_info param for rescaling",
                      info_.width, info_.height, msg->width, msg->height);
      }
    }
    info_.header.stamp = now;
    info_.header.frame_id = frame_id_;

    /* Publish ROS messages */
    image_pub_.publish(*msg, info_);
    mono_image_pub_.publish(mono_msg);

    ros::spinOnce(); // only necessary if there are callbacks
    loop_rate.sleep();

    if (show_image_)
    {
      /* Show image */
    	cv::imshow(windowName.c_str(), colorImg);
      char key = cv::waitKey(1);

    	/* Keyboard options */
    	switch (key) 
      {
      	/* When press the 'q' key then quit. */
      	case 'q':
      		show_image_ = false;
      		break;

      	/* When press the '[' key then decrease the exposure time. */
      	case '[':
          if (exposure > 1)
          {
      		  camera.set_control("Exposure (Absolute)", --exposure);
            std::cout << "oCam exposure: " << exposure << "\n";
          }
      		break;

  		  /* When press the ']' key then increase the exposure time. */
      	case ']':
          if (exposure < 625)
          {
      		  camera.set_control("Exposure (Absolute)", ++exposure);
            std::cout << "oCam exposure: " << exposure << "\n";
          }
      		break;

  		  /* When press the '-' key then decrease the brightness. */
      	case '-':
      		if (brightness > 0)
          {
      		  camera.set_control("Gain", --brightness);
            std::cout << "oCam brightness: " << brightness << "\n";
          }
      		break;

  		  /* When press the '=' key then increase the brightness. */
      	case '=':
      		if (brightness < 127)
          {
      		  camera.set_control("Gain", ++brightness);
            std::cout << "oCam brightness: " << brightness << "\n";
          }
      		break;

      	default:
      		break;
    	}
    }
  }

  cv::destroyAllWindows();

  /*
   * Stop streaming
   */
  camera.stop();

  ROS_INFO("Ocam Done.\n");
}

// Rescaling taken from cv_camera package. See http://wiki.ros.org/cv_camera for ROS 
// package info and https://github.com/OTL/cv_camera for source.
void oCam_ROS::rescaleCameraInfo(int width, int height)
{
  double width_coeff = width / info_.width;
  double height_coeff = height / info_.height;
  info_.width = width;
  info_.height = height;

  // See http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html for clarification
  info_.K[0] *= width_coeff;
  info_.K[2] *= width_coeff;
  info_.K[4] *= height_coeff;
  info_.K[5] *= height_coeff;

  info_.P[0] *= width_coeff;
  info_.P[2] *= width_coeff;
  info_.P[5] *= height_coeff;
  info_.P[6] *= height_coeff;
}

oCam_ROS::~oCam_ROS()
{
  delete camera_;
}


/*
 * Start the node
 */
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "ocam_ros_node");
  oCam_ROS thing;
  return 0;
}
