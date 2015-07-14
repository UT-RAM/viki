#include <AprilTags/TagDetector.h>
#include <geometry_msgs/Quaternion.h>

#include <cmath>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

/**
* Normalize angle to be within the interval [-pi,pi].
*/
double standardRad(double t) {
    if (t >= 0.) {
        t = fmod(t+PI, TWOPI) - PI;
    } else {
        t = fmod(t-PI, -TWOPI) + PI;
    }
    return t;
}

/**
* Convert rotation matrix to Euler angles
*/
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}



/**
  * Convert euler to rotation matrix
  * from: http://planning.cs.uiuc.edu/node102.html
  * a = yaw, b = pitch, c = roll
  */
Eigen::Matrix3f euler_to_rotMat(float a, float b, float c) {
    Eigen::Matrix3f R;

    R << cos(a)*cos(b), cos(a)*sin(b)*sin(c) - sin(a)*cos(c), cos(a)*sin(b)*cos(c) + sin(a)*sin(c),
         sin(a)*cos(b), sin(a)*sin(b)*sin(c) + cos(a)*cos(c), sin(a)*sin(b)*cos(c) - cos(a)*sin(c),
         - sin(b)     , cos(b) * sin(c)                     , cos(b)* cos(c);

    return R;
}
void quaternion_to_euler(geometry_msgs::Quaternion q, Eigen::Matrix<float, 1, 3> euler) {
    double threshold = q.x * q.y + q.z * q.w;
    double yaw, pitch, roll;
    if (threshold >= .5) {        // singularity at north pole
        yaw     = 2 * atan2(q.x, q.w);
        pitch   = PI / 2;
        roll    = 0;
    } else if (threshold <= .5) { //singularity at south pole
        yaw     = -2 * atan2(q.x, q.w);
        pitch   = -PI / 2;
        roll    = 0;
    } else {
        double sqx = q.x * q.x;
        double sqy = q.y * q.y;
        double sqz = q.z * q.z;

        //TODO: Check this!
        yaw     = atan2(2*q.y*q.w-2*q.x*q.z , 1 - 2*sqy - 2*sqz);
        pitch   = asin(2 * threshold);
        roll    = atan2(2*q.x*q.w-2*q.y*q.z , 1 - 2*sqx - 2*sqz);
    }

    euler = Eigen::Matrix<float, 1, 3>();
    euler << yaw, pitch, roll;

    return;
}

Eigen::Quaternionf quaternion_multiply(Eigen::Quaternionf q1, Eigen::Quaternionf q2) {
    float w = (q1.w() * q2.w() - q1.x()*q2.x() - q1.y()*q2.y() - q1.z()*q2.z());
    float x = (q1.w() * q2.x() + q1.x()*q2.w() + q1.y()*q2.z() - q1.z()*q2.y());
    float y = (q1.w() * q2.y() - q1.x()*q2.z() + q1.y()*q2.w() + q1.z()*q2.x());
    float z = (q1.w() * q2.z() + q1.x()*q2.y() - q1.y()*q2.x() + q1.z()*q2.w());

    Eigen::Quaternionf Q(w,x,y,z);
    return Q;
}
