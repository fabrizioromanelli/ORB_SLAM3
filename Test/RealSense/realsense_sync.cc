/**
* Test video stream with ORB-SLAM2.
*
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>

#include "realsense.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
  try {
    RealSense realsense(RealSense::IMU_IR_STEREO);
    // realsense.enableLaser(40.0);

    cout << endl << "-------" << endl;
    cout << "Start processing video stream ..." << endl;

    realsense.getMotionFrequency();
    getchar();

    // Main loop
    // for(;;)
    // {
    //   realsense.run();

    //   if (mode == RealSense::IMU_IR_STEREO) {
    //     // cout << fixed << setw(11) << setprecision(6) << "IRD Timestamp   : " << realsense.getIRLeftTimestamp() << endl;
    //     cv::Mat irLeftMatrix  = realsense.getIRLeftMatrix();
    //     cv::Mat irRightMatrix = realsense.getIRRightMatrix();
    //     cv::Point3f acc;
    //     cv::Point3f gyro;

    //     rs2_time_t IR_timestamp = realsense.getIRLeftTimestamp();
    //     while(realsense.getGyroTimestamp() <= IR_timestamp)
    //     {
    //       // cout.precision(17);
    //       // cout << realsense.getGyroTimestamp() << " " << IR_timestamp << endl;
    //       acc  = realsense.getAccFrames();
    //       gyro = realsense.getGyroFrames();
    //       realsense.updateIMU();
    //     }
    //   }
    //   int key = waitKey(10);
    // }
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }
  return 0;
}