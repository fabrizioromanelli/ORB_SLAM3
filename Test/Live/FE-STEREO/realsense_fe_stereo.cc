/**
* Test video stream with ORB-SLAM3.
*
*/

#include <iostream>
#include <algorithm>
#include <fstream>
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
    cerr << endl << "Usage: ./realsense_fe_stereo" << endl 
                 << "         path_to_vocabulary" << endl
                 << "         path_to_configuration" << endl
                 << "         mode[FE_STEREO]" << endl
                 << "         display[ON/OFF]" << endl
                 << "         save images files[ON/OFF]" << endl
                 << "         auto close after loop closure[ON/OFF]" << endl
                 << "         print camera trajectory[ON/OFF]" << endl;
    return 1;
  }

  try {
    RealSense::sModality mode = RealSense::FE_STEREO;
    if (strcmp(argv[3], "FE_STEREO") == 0)
      mode = RealSense::FE_STEREO;

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

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    System SLAM(argv[1], argv[2], System::STEREO, display);

    cout << endl << "-------" << endl;
    cout << "Start processing video stream ..." << endl;

    if (saveFile)
    {
      int dir_err = mkdir("infrared", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
      dir_err = mkdir("depth", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }

    // Main loop
    for(;;)
    {
      realsense.run();

      if (mode == RealSense::FE_STEREO) {
        // cout << fixed << setw(11) << setprecision(6) << "FE Timestamp   : " << realsense.getFETimestamp() << endl;
        cv::Mat feLeftMatrix  = realsense.getFELeftMatrix();
        cv::Mat feRightMatrix = realsense.getFERightMatrix();

        // Pass the IR Left and Depth images to the SLAM system
        cv::Mat cameraPose = SLAM.TrackStereo(feLeftMatrix, feRightMatrix, realsense.getFETimestamp());

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

    // Save camera trajectory
    // SLAM.SaveTrajectory("CameraTrajectory.dat");
    // SLAM.SaveKeyFrameTrajectory("KeyFrameTrajectory.dat");
  }
  catch(exception& ex) {
    cout << ex.what() << endl;
  }

  return 0;
}

