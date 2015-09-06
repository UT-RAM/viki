/**
  * Class DotsDetector
  *
  * This is a refactoring of the original duavracv code from Jort Baarsma
  * So this can be used in the ram_tracker_buddy package as well
  *
  * April 2015 - Robin Hoogervorst - RaM
  */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>
#include <fstream>		// save to files
#include <iterator>     // std::ostream_iterator
#include <vector>       // std::vector
#include <algorithm>    // std::copy
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include <ctime>

#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/framevel.hpp>
#include <kdl/framevel_io.hpp>
#include "kdl_conversions/kdl_msg.h"
#include <random_numbers/random_numbers.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "duavracv_vision/SortedMarker.h"

using namespace cv;
using namespace std;

struct Marker
{
  Point2f MarkerPoint;
  double Distance;
  double CompValue;
  Marker(double newDistance, double newCompValue, Point2f newMarkerPoint)
  {
    Distance=newDistance;
    CompValue=newCompValue;
    MarkerPoint=newMarkerPoint;
  }
};

bool cmp( const Marker * a, const Marker * b )
{
  return a->CompValue < b->CompValue ;
}

double Pi=3.14159;

class DotsDetector : public AbstractDetector
{


public:
  //! The Constructor
  DotsDetector();
  //! The Destructor
  ~DotsDetector();

  int FilterSize;

  vector<double> rvec;
  vector<vector<double> > rvec_filter;
  vector<double> rvec_filtered;
  //! Camera pose : Transation.
  vector<double> tvec;


  //! Is set to true if the pose is succesfully found.
  bool PoseFound;
  bool TextDebugEnabled;

  //! The desired height/width ratio for the inner contours
  double DesiredRatio;

  bool findPosition(cv::Mat raw_image);
private:
  cv_bridge::CvImagePtr cv_ptr_undist;
  Mat AugmentedReality;
  vector<Mat> MovingAverage;
  //! The file stream used for logging.
  ofstream loggerfile;

  //! The time that is initialized when object is created.
  clock_t clock_start;
  //! The time when previous detecting iteration was done.
  clock_t clock_previous;

  //! The numbers of markers that make the reference object.
  int NrMarkers;
  //! The points of the reference markers (object).
  vector<Point3f> object_points;					//
  //! Four points that are used to make the axis.
  vector<Point3f> axis_points;						//
  //! The last known position of the markers.
  vector<Point2f> marker_points;					//

};


/**
  * HSV Dots detector, original created by Jort Baarsma for the duavracv vision project within RAM,
  * Adapted and cleaned up for easier usage
  */

DotsDetector::DotsDetector()
{
    NrMarkers           = 8;			// Initialize the number of markers
    DesiredRatio        = 1.5;
    clock_start         = clock();		// Initialize the clock
    TextDebugEnabled    = false;		// Initialize the Debug variable for command line output.

    FilterSize=5;
    for (int i=0;i<FilterSize;i++)
    {
      rvec_filter.push_back(rvec);
    }

    while(marker_points.size()<NrMarkers)
    {
      marker_points.push_back(Point2f(1.0f, 5.0f));
    }

    rvec.push_back(0.0f); rvec.push_back(0.0f); rvec.push_back(0.0f);
    rvec_filtered.push_back(0.0f); rvec_filtered.push_back(0.0f); rvec_filtered.push_back(0.0f);
    tvec.push_back(0.0f); tvec.push_back(0.0f); tvec.push_back(0.0f);


    // Initialize marker points
    double theta_angle=28/2; // Between 24 and 28 degree
    double theta=3.14159*theta_angle/180;
    double U=30E-3;
    double V=26.6666667E-3;
    object_points.push_back(Point3f(-(V/2)*cos(theta),		U, (V/2)*sin(theta)));
    object_points.push_back(Point3f(-(3*V/2)*cos(theta),	U, (3*V/2)*sin(theta)));
    object_points.push_back(Point3f(-(3*V/2)*cos(theta),	-U, (3*V/2)*sin(theta)));
    object_points.push_back(Point3f( (V/2)*cos(theta),-		U, (V/2)*sin(theta)));
    object_points.push_back(Point3f( (3*V/2)*cos(theta),	-U, (3*V/2)*sin(theta)));
    object_points.push_back(Point3f( (3*V/2)*cos(theta),	0, (3*V/2)*sin(theta))); // A-symmetric one
    object_points.push_back(Point3f( (3*V/2)*cos(theta),	U,  (3*V/2)*sin(theta)));
    object_points.push_back(Point3f(0.0f, 0.0f, 0.0f));																		// ZERO

}

DotsDetector::~DotsDetector() {};

bool DotsDetector::findPosition(cv::Mat raw_image)
{


    //------------------------------------------------
    // --------------- Main code block ---------------
    //------------------------------------------------
    bool FoundMarker=false;
    PoseFound = false;
    Mat Outer_Image;
    Mat Outer_HSV; 			// The Markers as binary output after inner HSV filtering
    Mat Outer_Morph; 		// Variable for the Morphological operations on outer HSV image
    Mat Inner_HSV; 			// The Markers as binary output after inner HSV filtering
    Mat Inner_Morph;		// Variable for the Morphological operations on inner HSV image
    Mat BoundBoxOutput;
    vector< vector<Point> > Outer_contours;
    vector< vector<Point> > Inner_contours;
    vector<Point> approx;

    // --------------- IMAGE MOVING AVERAGE --------------------
    while (MovingAverage.size()<2)
    {
      MovingAverage.push_back(raw_image);
    }
    MovingAverage.insert(MovingAverage.begin(),raw_image);
    MovingAverage.pop_back();
    raw_image = MovingAverage[0];


    // --------------- OUTER HSV FILTERING --------------------
    // Used to calculate areas where the markers could be located
    // Filters orange/yellow at the moment.
    {
      Mat HSV_image;
      cvtColor(raw_image, HSV_image, COLOR_BGR2HSV);
      int range=12;    // 8
      int hue=168;	   // 172
      int sat=40;      // 65
      int val=40;      //65
      Scalar lower_color = Scalar(hue-range,sat,val);
      Scalar upper_color = Scalar(hue+range,256,256);
      inRange(HSV_image, lower_color, upper_color, Outer_HSV);
    }

    // --------------- OUTER MORPHOLOGICAL FILTERING --------------------
    // Used to filter out the noise and close the found markers area.
    // First erodes and then dilates this is called 'closing'.
    {
      int erosion_size=3;
      Mat element = getStructuringElement( MORPH_ELLIPSE,Size( 2*erosion_size + 1, 2*erosion_size+1 ), Point( erosion_size, erosion_size ) );
      erode( Outer_HSV, Outer_Morph, element );

      int dilate_size=10;
      element = getStructuringElement( MORPH_ELLIPSE,Size( 2*dilate_size + 1, 2*dilate_size+1 ), Point( dilate_size, dilate_size ) );
      dilate( Outer_Morph, Outer_Morph, element );
      // Now apply the Morph
      bitwise_and(raw_image,raw_image,Outer_Image,Outer_Morph);
    }

    // --------------- OUTER CONTOUR FILTERING --------------------
    // Now we find and define the contours that are found in the remaining image.
    findContours( Outer_Morph, Outer_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    // --== Pre Loop Variable generation ==--
    double Scale=1;
    vector<Rect> boundRect( Outer_contours.size() );
    int ROI_Nr = 0;
    int Blobs = 0;
    int nrOfEllipses=0;

    // ..:: OUTER CONTOURS LOOPS ::..
    for( int i=0; i < Outer_contours.size(); i++ )
    {
      Scale=1;

      // Normal bounding box (non-rotated)
      boundRect[ROI_Nr] = boundingRect(Outer_contours[i]); // Top left (x,y) and width and height
      ROI_Nr++;

      BoundBoxOutput = raw_image(Rect(boundRect[i].x,boundRect[i].y,boundRect[i].width,boundRect[i].height)).clone();

      // --------------- GAUSSIAN BLUR FILTERING --------------------
      // Far away noise can more easily disturb the markers and blur helps.
      // Is only applied when the marker is larger then a certain area.
      double BoundedArea=boundRect[i].width*boundRect[i].height;
      if(BoundedArea>40000) // 200 x 200 pixels
      {
        GaussianBlur(BoundBoxOutput,BoundBoxOutput,Size(3,3),0,0,0);
      }
      else if (BoundedArea>22500)// 150 x 150 pixels
      {
        Scale=2;
        resize(BoundBoxOutput, BoundBoxOutput, Size(), Scale, Scale);
        GaussianBlur(BoundBoxOutput,BoundBoxOutput,Size(3,3),0,0,0);
      }
      else
      {
        //resize(BoundBoxOutput, BoundBoxOutput, Size(), 2, 2, INTER_CUBIC);
        Scale=3;
        resize(BoundBoxOutput, BoundBoxOutput, Size(), Scale, Scale);
        GaussianBlur(BoundBoxOutput,BoundBoxOutput,Size(5,5),0,0,0);
      }

      // --------------- INNER HSV FILTERING --------------------
      // Used to calculate areas where the markers could be located
      // Filters orange/yellow at the moment.

      {
        Mat BoundBoxHSV; // Temporary HSV image used for filtering [only in this scope]

        int RedRange  = 8;
        int RedHue    = 23;
        int RedSat    = 65;
        int RedVal    = 65;
        cvtColor(BoundBoxOutput, BoundBoxHSV, COLOR_BGR2HSV);

        Scalar lower_color =Scalar(RedHue-RedRange,RedSat,RedVal);
        Scalar upper_color =Scalar(RedHue+RedRange,256,256);
        inRange(BoundBoxHSV, lower_color, upper_color, Inner_HSV);
      }

      // --------------- INNER MORPHOLOGICAL FILTERING --------------------
      // Used to filter out the noise and close the found markers area.
      // First erodes and then dilates this is called 'closing'.
      {
        int erosion_size=3;
        Mat element = getStructuringElement( MORPH_ELLIPSE,Size( 2*erosion_size + 1, 2*erosion_size+1 ), Point( erosion_size, erosion_size ) );
        erode( Inner_HSV, Inner_Morph, element );

        int dilate_size=5;
        element = getStructuringElement( MORPH_ELLIPSE,Size( 2*dilate_size + 1, 2*dilate_size+1 ), Point( dilate_size, dilate_size ) );
        dilate( Inner_Morph, Inner_Morph, element );
      }

      // --------------- INNER CONTOUR FILTERING --------------------
      // Now we find and define the contours that are found in the remaining image.
      findContours( Inner_Morph, Inner_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

      // --== Pre Loop Variable generation ==--
      vector<RotatedRect> minEllipse( Inner_contours.size() );
      nrOfEllipses = 0;
      // ..:: INNER CONTOURS LOOPS ::..
      for( int n=0; n < Inner_contours.size(); n++ )
      {
          Blobs++;
          // Approximate contour with accuracy proportional to the contour perimeter
          approxPolyDP( 	Mat(Inner_contours[n]),approx,arcLength( Mat(Inner_contours[n]), true) * 0.01,true );

          // Calculate the height / width = ratio
          double Ratio;
          Rect Temp=boundingRect(Inner_contours[n]);
          Ratio=(double) Temp.width/(double) Temp.height;

          if( approx.size() > 4 && Ratio<DesiredRatio && Ratio > (1/DesiredRatio)  )
          {
              minEllipse[nrOfEllipses] = fitEllipse( Mat(Inner_contours[n]) );
              nrOfEllipses++;
          }
      }

      // Actual marker detection
      if (NrMarkers==8)
      {
      Point2f UnsortedMarker[NrMarkers];
      if ( nrOfEllipses == NrMarkers )// NrMarkers )// General so not to limit
      {
        FoundMarker=true;

        if (TextDebugEnabled) {cout << NrMarkers << " markers detected!" << endl;}

        // --== Pre Loop Variable generation ==--
        Point CenterPoint=Point(0,0);			// Make a empty centre point with which to calculate.
        double Fade=255;
        // ..:: Loop through all the markers ::..
        for ( int n=0; n < NrMarkers; n++)
        {
          // Add the location of the bounding box to the marker locations
          UnsortedMarker[n].x=(double)((double)minEllipse[n].center.x/Scale+(double)boundRect[i].x);
          UnsortedMarker[n].y=(double)((double)minEllipse[n].center.y/Scale+(double)boundRect[i].y);
          // Re-use the marker to determine the average point (center)
          CenterPoint.x+=UnsortedMarker[n].x/NrMarkers;
          CenterPoint.y+=UnsortedMarker[n].y/NrMarkers;

          //Green Centerpoints of the circles
          Fade=Fade-255/(NrMarkers-1);
        }

        // --== Pre Loop Variable generation ==--
        int ClosestMarker;
        double Distance=0;
        double ClosestDistance=65535;

        // ..:: Loop through all the markers [again] ::..
        for ( int n=0; n < NrMarkers; n++)	// Sort the 4 detected markers clockwise
        {
          Distance=sqrt(pow(UnsortedMarker[n].x-CenterPoint.x,2)+pow(UnsortedMarker[n].y-CenterPoint.y,2));
          if (ClosestDistance > Distance)
          {
              ClosestDistance=Distance;
              ClosestMarker=n;
          }
        }

        double vector_x=CenterPoint.x-UnsortedMarker[ClosestMarker].x;
        double vector_y=CenterPoint.y-UnsortedMarker[ClosestMarker].y;
        double theta=atan2(vector_y, vector_x);

        double CompValue=0;
        vector <Marker*> v;
        for (int n=0; n<NrMarkers;n++)
        {
            if( n!=ClosestMarker)
            {
                double vector_x=UnsortedMarker[n].x-UnsortedMarker[ClosestMarker].x;
                double vector_y=UnsortedMarker[n].y-UnsortedMarker[ClosestMarker].y;;
                double temp_x=vector_x;
                double temp_y=vector_y;

                CompValue=atan2(temp_y,temp_x);
                v.push_back(new Marker(0,CompValue,UnsortedMarker[n]));
            }
        }

        sort( v.begin(), v.end(), cmp );

        int RequiredShift=0;
        int Counter=0;
        double Epsilon=Pi/8;

        for (int m=0; m<6; m++)
        {
            double Phi = (v[m+1]->CompValue-v[m]->CompValue);

            if (abs(Phi) > Pi) // First and last are compared. Fix the 2Pi issue
            {
                Phi=signbit(Phi)*2*Pi-Phi;
            }

            if (abs(Phi) < (Pi/4)) // Skip
            {
              Counter++;
              if (Counter==3)
                {
                  RequiredShift=m+2;
                  m=8; // STOP LOOP
                }
            }
            else								// Go
            {
              Counter=0;
            }
        }

        if (Counter==1) {RequiredShift=2; }
          else if (Counter==2) {RequiredShift=1; }
          else if (Counter==0) {RequiredShift=4; }

        std::rotate(v.begin(), v.begin() + RequiredShift, v.end()); // Left
        //std::rotate(v.rbegin(), v.rbegin() + RequiredShift, v.rend()); // Right
        v.push_back(new Marker(0,10,UnsortedMarker[ClosestMarker]));


        for ( int n=0; n < NrMarkers; n++)	// Sort the 4 detected markers clockwise
        {
            marker_points[n].x=(double) v[n]->MarkerPoint.x;
            marker_points[n].y=(double) v[n]->MarkerPoint.y;
              // if (TextDebugEnabled) {cout << "SORTED MARKER VECTOR:" << (marker_points[n].x-marker_points[NrMarkers-1].x) << "," << (marker_points[n].y-marker_points[NrMarkers-1].y) << endl;}
              // if (TextDebugEnabled) {cout << "SORTED OBJECT VECTOR:" << object_points[n].x << "," << object_points[n].y << endl;}
        }


        // Calculate the Pose and Position using the given OpenCV function.
        solvePnP(object_points, marker_points, calibration->cameraMatrix, calibration->distCoeffs,
                  rvec, tvec,false);

        PoseFound=true;
        if (TextDebugEnabled)
          {
            cout << "Contour :[" << ROI_Nr << "], has ["  << Blobs  << "] Blobs and [" << nrOfEllipses << "] circles." << endl;
          }

          // --------------- LOOP BREAKER ---------------
          // Breaks the loop if the marker is found, no other contours are checked.
          if (FoundMarker)
            {
              i = Outer_contours.size();
            }
          } // ..:: INNER CONTOUR LOOP ENDS ::..

        }
        else
        {
          FoundMarker=false;
        }

      } // ..:: OUTER CONTOUR LOOP ENDS ::..

      Mat Rot;
      double xrot;
      double yrot;
      double zrot;
      if(PoseFound)
      {
          // --------------- REDEFINING THE ROTATION --------------------
          // Now we find and define the contours that are found in the remaining image.
          Rodrigues(rvec, Rot);

          yrot = (double) atan2(Rot.at<double>(0,2), Rot.at<double>(2,2));
          xrot = (double) asin(-Rot.at<double>(2,1));
          zrot = (double) atan2(Rot.at<double>(0,1),Rot.at<double>(1,1));

          //Calculate the angle in degrees
          yrot = yrot * (double) (180.0 / CV_PI)-180;
          xrot = xrot * (double) (180.0 / CV_PI);
          zrot = zrot * (double) (180.0 / CV_PI)-180;
          if (TextDebugEnabled) {cout << "[" << tvec[0] << "," << tvec[1] << "," << tvec[2] << "]"  << endl;}

          // Check if the difference is small enough
          float dx = position[0] - tvec[2];
          float dy = position[1] + tvec[0]; // double negative
          float dz = position[2] + tvec[1];
          float diff = dx * dx + dy * dy + dz * dz;

          // Set she position from our posittion
          position << tvec[2],-tvec[0],-tvec[1];
          if (diff > .5) return false;
      } else {
          return false;
      }
      return PoseFound;

      if (TextDebugEnabled)
      {
        if(PoseFound)
        {
            cout << "Rotations = x: " << xrot << " y= " << yrot << " z= " << zrot << endl;
            cout << "Translation = x: " << tvec[0] << " y= " << tvec[1] << " z= " << tvec[2] << endl;
        }
      }
}
