/**
* Test video stream with ORB-SLAM2.
*
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <thread>
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

    cv::Mat irLeftMatrix, irRightMatrix;
    std::vector<std::vector<double>> gyro_measurements, acc_measurements;
    double vFrameTs = 0.0;
    cout << endl << "-------" << endl;
    cout << "Start processing video stream ..." << endl;

    std::thread grabThread(&RealSense::startGrab, &realsense);

    std::cout.precision(17);

    while(1) {
      if (realsense.getFrames(irLeftMatrix, irRightMatrix, gyro_measurements, acc_measurements, vFrameTs))
      {
        std::cout << irLeftMatrix.size << std::endl;
        std::cout << irRightMatrix.size << std::endl;
        std::cout << vFrameTs << std::endl;
        std::cout << "GYRO:" << std::endl;
        for (size_t i = 0; i < gyro_measurements.size(); i++)
          std::cout << gyro_measurements[i][0] << " " << gyro_measurements[i][1] << " " << gyro_measurements[i][2] << " " << gyro_measurements[i][3] << " " << std::endl << std::flush;

        std::cout << "ACC:" << std::endl;
        for (size_t i = 0; i < acc_measurements.size(); i++)
          std::cout << acc_measurements[i][0] << " " << acc_measurements[i][1] << " " << acc_measurements[i][2] << " " << acc_measurements[i][3] << " " << std::endl << std::flush;
      }
      usleep(20000);
    }
    getchar();
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }
  return 0;
}