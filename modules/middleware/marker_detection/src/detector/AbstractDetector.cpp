#include <Eigen/LU>

/**
  *   Abstract class for a image detector within ROS.
  *   An extended class of this should export a position relative to the camera,
  *   using the settings and functions provided here
  *
  *   April 2015 - Robin Hoogervorst - RaM
  */

struct DetectorCameraCalibration {
    cv::Mat cameraMatrix, distCoeffs;
    cv::Size image_size;

    cv::Point2f fov;    // focal length
    cv::Point2f pp;     // princpal point
};

class AbstractDetector {
protected:
  DetectorCameraCalibration* calibration;

  Eigen::Vector3f     position;     // Position found in camera frame
  Eigen::Quaternionf  orientation;  // Orientation found in camera frame
  Eigen::Vector2f     detectionPoint; // x, y coordinates in pixel frame (where has the image been found)

  bool debug;

public:
  AbstractDetector() {
    calibration = new DetectorCameraCalibration;

    Eigen::Vector2f detectionPoint(0, 0);
  }
  ~AbstractDetector() {
    delete(calibration);
  }

  /**
    * Find the position of a marker, using a openCV Mat image.
    * This position is relative to the camera (z forward, x right, y down) (Same as openCV solvePnP)
    * Then it should populate the position and orientation variables in this detector,
    * which can be get later on.
    *
    * @return bool Was a position found?
    */
  virtual bool findPosition(cv::Mat image) {};

  Eigen::Vector3f     getPosition()     { return position;    };
  Eigen::Quaternionf  getOrientation()  { return orientation; };
  Eigen::Vector2f     getDetectionPoint()  { return detectionPoint; };

  /**
    * Parses the camera settings from an xml file,
    * this can be generated using the opencv calibration functionality
    *
    * Will populate the calibration sturct in this class, so that can be used
    */
  void parseCameraSettings(std::string filename) {
      cout << "Parsing camera settings..";

      // Read the file configuration
      cv::FileStorage fs ( filename, cv::FileStorage::READ );

      fs["Camera_Matrix"]           >> calibration->cameraMatrix;
      fs["Distortion_Coefficients"] >> calibration->distCoeffs;
      fs["Image_Width"]             >> calibration->image_size.width;
      fs["Image_Height"]            >> calibration->image_size.height;

      double apertureWidth=4.54E-3, apertureHeight=3.42E-3;
      double fovx, fovy, focalLength, aspectRatio;
      cv::Point2d principalPoint;

      cv::calibrationMatrixValues ( calibration->cameraMatrix, calibration->image_size, apertureWidth, apertureHeight, //input
                                    fovx, fovy, focalLength, principalPoint, aspectRatio); // output

      calibration->fov.x = fovx;
      calibration->fov.y = fovy;
      calibration->pp.x = principalPoint.x;
      calibration->pp.y = principalPoint.y;

      fs.release();
  }

  /**
    * Decorate an image with an axis on the found marker
    * Projects a set of axis on a Mat CV image
    */
  void decorateImage(cv::Mat& image) {

    // Define the axis points (6cm long)
    vector<cv::Point3f> axis_points;
    axis_points.push_back(cv::Point3f(0.0f, 0.0f, 0.0f));
    axis_points.push_back(cv::Point3f(0.06f, 0.0f, 0.0f));
    axis_points.push_back(cv::Point3f(0.0f, 0.06f, 0.0f));
    axis_points.push_back(cv::Point3f(0.0f, 0.0f, 0.06f));
    vector<cv::Point2f> proj_axis_points(4);

    // Generate position and rotation in the right form for openCV
    vector<double> tvec(3);
    tvec[0] = position(0);
    tvec[1] = position(1);
    tvec[2] = position(2);

    vector<double> rvec(3);
    cv::Matx33d r = cv::Mat(3, 3, CV_64F, double(0));
    Eigen::Matrix3f O = orientation.toRotationMatrix();
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        r(i,j) = O(i,j);
      }
    }
    Rodrigues(r, rvec);

    // Calculate the right points using opencv
    projectPoints( axis_points, rvec, tvec,
              calibration->cameraMatrix, calibration->distCoeffs,
              proj_axis_points);

    // Draw the found positions on the image
    line(image, proj_axis_points[0], proj_axis_points[1], cv::Scalar(0,0,255), 2, 8, 0 ); 	//Red x-axis
    putText(image, "X", proj_axis_points[1], cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,0,255), 1, CV_AA);

    line(image, proj_axis_points[0], proj_axis_points[2], cv::Scalar(255,0,0), 2, 8, 0 );	//cv::Scalar(0,255,0) Blue y-axis
    putText(image, "Y", proj_axis_points[2], cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255,0,0), 1, CV_AA);

    line(image, proj_axis_points[0], proj_axis_points[3], cv::Scalar(0,255,0), 2, 8, 0 );	//Green z-axis
    putText(image, "Z", proj_axis_points[3], cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 1, CV_AA);
  }

};
