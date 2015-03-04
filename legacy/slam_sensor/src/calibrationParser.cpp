/* The MIT License (MIT)

Copyright (c) 2015 Rutger Hendriks & Roald Looge

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

#include "calibrationParser.h"

/**
    Constructor ImageManipulator

    Gets the path to the package and parses the calibration data
*/
CalibrationParser::CalibrationParser()
{
    filename = ros::package::getPath("SLAM_sensor");
    filename += "/camera_calibration.yaml";

    if (!parseCalibration())
    {
        // BAD
		ROS_ERROR("Parsing calibration was not succesful");
    }
}


/**
    Opens the calibration file and reads in the data into the struct array 'cams'
    Returns:
            bool - returns if the reading in was succesfull or not
*/
bool CalibrationParser::parseCalibration()
{
    YAML::Node config = YAML::LoadFile(filename);
    parseYAML(config["cam0"], cams[0]);
    parseYAML(config["cam1"], cams[1]);

    return true;
}

/**
    Checks if the calibration data is already parsed,
    and puts it into sensor_msgs::CameraInfo format
    Parameters:
        cam_info[] - Array consisting of 2 CameraInfo where the data will be put in
    Returns:
        bool - returns if the reading in was succesfull or not
*/
sensor_msgs::CameraInfo* CalibrationParser::parseCalibrationCameraInfo(sensor_msgs::CameraInfo *cam_info_[])
{

		// Dereferencing
		static sensor_msgs::CameraInfo cam_info[2];
		cam_info[0] = *cam_info_[0];
		cam_info[1] = *cam_info_[1];		

        // Make sure it's already parsed
        if (cams[0].width == 0) parseCalibration();

        for (unsigned i=0; i<2; i++)
        {
            cam_info[i].height = cams[i].height;

            cam_info[i].width = cams[i].width;
            cam_info[i].distortion_model = cams[i].distortion_model;
            cam_info[i].D = std::vector<double>(cams[i].D, cams[i].D + sizeof(cams[i].D) / sizeof(double));

            // Work around for handling boost::Array
            for (unsigned j=0; j<cam_info[i].K.size(); j++)
            {
                cam_info[i].K[j] = cams[i].K[j];
            }

            for (unsigned j=0; j<cam_info[i].R.size(); j++)
            {
                cam_info[i].R[j] = cams[i].R[j];
            }

            for (unsigned j=0; j<cam_info[i].P.size(); j++)
            {
                cam_info[i].P[j] = cams[i].P[j];
            }
        }
		return cam_info;
}


void CalibrationParser::parseYAML(const YAML::Node& node, CalibrationParser::Cam_cal& c)
{
    c.width = node["image_width"].as<int>();
    c.height = node["image_height"].as<int>();
    c.distortion_model = node["distortion_model"].as<std::string>();

    const YAML::Node& n1 = node["camera_matrix"]["data"];
    for (unsigned i=0; i < n1.size(); i++)
    {
        c.K[i] = n1[i].as<double>();
    }

    const YAML::Node& n2 = node["projection_matrix"]["data"];
    for (unsigned i=0; i < n2.size(); i++)
    {
        c.P[i] = n2[i].as<double>();
    }

    const YAML::Node& n3 = node["rectification_matrix"]["data"];
    for (unsigned i=0; i < n3.size(); i++)
    {
        c.R[i] = n3[i].as<double>();
    }

    const YAML::Node& n4 = node["distortion_coefficients"]["data"];
    for (unsigned i=0; i < n4.size(); i++)
    {
        c.D[i] = n4[i].as<double>();
    }

    // Extra options for parsing rotation and translation
    // These values are only given with the second camera parameters
    if (node["rotation_matrix"]["data"])
    {
        const YAML::Node& n5 = node["rotation_matrix"]["data"];
        for (unsigned i=0; i < n5.size(); i++)
        {
            transf_.rot[i] = n5[i].as<double>();
        }

        const YAML::Node& n6 = node["translation_matrix"]["data"];
        for (unsigned i=0; i< n6.size(); i++)
        {
            transf_.transl[i] = n6[i].as<double>();
        }
    }
}
