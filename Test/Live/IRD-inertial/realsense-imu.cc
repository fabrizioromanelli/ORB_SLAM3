/**
* Test video stream with ORB-SLAM2.
*
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <thread>
#include <chrono>
#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <System.h>
#include <sys/stat.h>

#include "realsense.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM3;

int main(int argc, char **argv)
{
  if(argc != 8)
  {
    cerr << endl << "Usage: ./realsense_live" << endl 
                 << "         path_to_vocabulary" << endl
                 << "         path_to_configuration" << endl
                 << "         mode[RGBD/IRD]" << endl
                 << "         display[ON/OFF]" << endl
                 << "         save images files[ON/OFF]" << endl
                 << "         auto close after loop closure[ON/OFF]" << endl
                 << "         print camera trajectory[ON/OFF]" << endl;
    return 1;
  }

  try {
    RealSense::sModality mode = RealSense::RGBD;
    if (strcmp(argv[3], "RGBD") == 0)
      mode = RealSense::RGBD;
    else if (strcmp(argv[3], "IMU_IRD") == 0)
      mode = RealSense::IMU_IRD;

    RealSense realsense(mode);
    // realsense.enableLaser(40.0);

    // Clone parameters from command line
    bool display = false;
    string displayS = string(argv[4]);
    if(displayS.compare("ON") == 0)
      display = true;

    bool saveFile = false;
    string saveFileS = string(argv[5]);
    if(saveFileS.compare("ON") == 0)
      saveFile = true;

    bool autoclose = false;
    string autocloseS = string(argv[6]);
    if(autocloseS.compare("ON") == 0)
      autoclose = true;

    bool printTraj = false;
    string printTrajS = string(argv[7]);
    if(printTrajS.compare("ON") == 0)
      printTraj = true;

    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    // Start grabbing frames from realsense
    std::thread grabThread(&RealSense::startGrab, &realsense);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    System SLAM(argv[1], argv[2], System::IMU_RGBD, display);
    Verbose::SetTh(Verbose::VERBOSITY_QUIET);

    cout << endl << "-------" << endl;
    cout << "Start processing video stream ..." << endl;

    if (saveFile)
    {
      int dir_err = mkdir("infrared", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      dir_err = mkdir("depth", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }

    cv::Mat irMatrix, depthMatrix;
    std::vector<std::vector<double>> gyro_m, acc_m;
    double vFrameTs = 0.0;

    // Main loop
    for(;;)
    {
      if (realsense.getFrames(irMatrix, depthMatrix, gyro_m, acc_m, vFrameTs)) {
        // Load imu measurements from previous frame
        vImuMeas.clear();
        // std::cout << "vFrameTs " << vFrameTs << std::endl << std::flush;
        for (size_t i = 0; i < acc_m.size(); i++)
        {
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc_m[i][1],acc_m[i][2],acc_m[i][3],gyro_m[i][1],gyro_m[i][2],gyro_m[i][3],acc_m[i][0]));
          // std::cout << acc_m[i][1] << " " << acc_m[i][2] << " " << acc_m[i][3] << " " << gyro_m[i][1] << " " << gyro_m[i][2] << " " << gyro_m[i][3] << " " << acc_m[i][0] << std::endl << std::flush;
        }

        // Pass the IR Left and Depth images to the SLAM system
        cv::Mat cameraPose = SLAM.TrackRGBD(irMatrix, depthMatrix, vFrameTs, vImuMeas);

        if (printTraj)
          cout << "Camera position" << cameraPose << endl;
      }

      int key = waitKey(10);
      // Stop SLAM when Spacebar is pressed or if the map changed (so a loop has been closed)
      if( key == 32 || (SLAM.MapChanged() && autoclose)) {
        cout << "Loop closed ==> shutting down SLAM" << endl;
        break;
      }
    }

    // Stop all threads
    SLAM.Shutdown();
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  return 0;
}

