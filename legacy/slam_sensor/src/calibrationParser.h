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

#pragma once
#include <ros/ros.h>
#include <ros/package.h>
#include <vector>

#include "sensor_msgs/CameraInfo.h"
#include "yaml-cpp/yaml.h"


class CalibrationParser
{
    public:
        CalibrationParser();
        ~CalibrationParser();

        struct Cam_cal                  // Struct for holding calibration data
        {
            int width, height;
            std::string distortion_model;
            double D[4];                // Distortion parameters
            double K[9];                // Intrinsic camera matrix 3x3
            double R[9];                // Rectification matrix 3x3
            double P[12];               // Projection matrix 3x4
        };

        struct Cam_transf
        {
            std::vector<double> rot;
            std::vector<double> transl;
        };

		sensor_msgs::CameraInfo* parseCalibrationCameraInfo(sensor_msgs::CameraInfo *cam_info[]);
        void parseYAML(const YAML::Node& node, CalibrationParser::Cam_cal& c);
        inline Cam_cal getCalibration(const int index) { return cams[index]; }
        inline std::vector<double> getTranslation() { return transf_.transl; }
        inline std::vector<double> getRotation() { return transf_.rot; }

    private:
        struct Cam_cal cams[2];
        struct Cam_transf transf_;
        std::string filename;
        bool parseCalibration();
};


