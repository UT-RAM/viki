/**
  * AprilDetector
  *
  * Bridge for the Apriltag library and the Ram_tracker_buddy camera_pose estimator
  * Extends the AbstractDetector, for easy swapping of classes
  *
  * getRelativeTransform has been reimplemented (basically copied from the original library),
  * since the original library didn't use distortion coefficients
  *
  * April 2015 - Robin Hoogervorst - RaM
  */

#include <ros/ros.h>

#include <AprilTags/TagDetector.h>
#include "AprilTags/Tag36h11.h"

class AprilDetector : public AbstractDetector {

private:
  boost::shared_ptr<AprilTags::TagDetector> tagDetector;

  double tagSize;

public:
  AprilDetector() {
    //initialize the detector
    AprilTags::TagCodes m_tagCodes = AprilTags::tagCodes36h11;
    tagDetector = boost::shared_ptr<AprilTags::TagDetector>(new AprilTags::TagDetector(m_tagCodes));

    tagSize = 0;
    debug = true;
  }

    void setDouble(std::string name, double value) {
        if (name == "tagsize") {
            tagSize = value;
        }
    }

  /**
    * Find a position and populate the right values in this class.
    *
    * WARNING: Will only publish if one tag has been found, since the whole
    * set-up is based on the detection of a single tag.  
    */
  bool findPosition(cv::Mat image) {
    // Apriltags needs a grayscale image as input
    cv::Mat image_gray;
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);

    // Call to detect the tags
    vector<AprilTags::TagDetection> detections = tagDetector->extractTags(image_gray);

    // Only publish when we have found one tag
    if (detections.size() == 1) {
        AprilTags::TagDetection detection = detections[0];

        // Get the 3d coordinates from the detection
        Eigen::Matrix4d Transform = getRelativeTransform(detection.p);

        // Extract translation and rotation from the transformation matrix
        Eigen::Vector3d translation = Transform.topRightCorner(3,1);
        Eigen::Matrix3d rotation    = Transform.topLeftCorner(3,3);

        // Populate our own variables
        // Eigen::Quaternionf orientation;
        orientation    = Eigen::Quaternionf(rotation.cast<float>());
        position       = translation.cast<float>();
        detectionPoint = Eigen::Vector2f(detection.cxy.first, detection.cxy.second);

        // Draw axis on the image, using the data we just found
        decorateImage(image);

        // We found something, yay! So we can return that :)
        return true;
    }

    // If nothing has been found (or too much). Return false to indicate nothing has been found
    return false;
  }

  /**
    * Get the 3d pose from the camera position found.
    *
    * Since the library does not include distortions into the solve PnP function,
    * the function has been copied and adapted in here.
    */
  Eigen::Matrix4d getRelativeTransform(std::pair<float, float> p[4]) {
      std::vector<cv::Point3f> objPts;
      std::vector<cv::Point2f> imgPts;
      double s = tagSize/2.;
      objPts.push_back(cv::Point3f(-s,-s, 0));
      objPts.push_back(cv::Point3f( s,-s, 0));
      objPts.push_back(cv::Point3f( s, s, 0));
      objPts.push_back(cv::Point3f(-s, s, 0));

      std::pair<float, float> p1 = p[0];
      std::pair<float, float> p2 = p[1];
      std::pair<float, float> p3 = p[2];
      std::pair<float, float> p4 = p[3];
      imgPts.push_back(cv::Point2f(p1.first, p1.second));
      imgPts.push_back(cv::Point2f(p2.first, p2.second));
      imgPts.push_back(cv::Point2f(p3.first, p3.second));
      imgPts.push_back(cv::Point2f(p4.first, p4.second));

      cv::Mat rvec, tvec;
      cv::solvePnP(objPts, imgPts, calibration->cameraMatrix, calibration->distCoeffs, rvec, tvec);
      cv::Matx33d r;
      cv::Rodrigues(rvec, r);
      Eigen::Matrix3d wRo;
      wRo << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);

      Eigen::Matrix4d T;
      T.topLeftCorner(3,3) = wRo;
      T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
      T.row(3) << 0,0,0,1;

      return T;
    }

};
