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
                 << "         mode[STEREO_IMU]" << endl
                 << "         display[ON/OFF]" << endl
                 << "         save images files[ON/OFF]" << endl
                 << "         auto close after loop closure[ON/OFF]" << endl
                 << "         print camera trajectory[ON/OFF]" << endl;
    return 1;
  }

  try {
    RealSense::sModality mode = RealSense::IMU_IR_STEREO;
    if (strcmp(argv[3], "STEREO_IMU") == 0)
      mode = RealSense::IMU_IR_STEREO;

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

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    System SLAM(argv[1], argv[2], System::IMU_STEREO, display);
    Verbose::SetTh(Verbose::VERBOSITY_VERY_VERBOSE);

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

      if (mode == RealSense::IMU_IR_STEREO) {
        // cout << fixed << setw(11) << setprecision(6) << "IRD Timestamp   : " << realsense.getIRLeftTimestamp() << endl;
        cv::Mat irLeftMatrix  = realsense.getIRLeftMatrix();
        cv::Mat irRightMatrix = realsense.getIRRightMatrix();
        cv::Point3f acc;
        cv::Point3f gyro;
        //cout << "X: " << acc.x << "Y: " << acc.y << "Z: " << acc.z << endl;

        // Load imu measurements from previous frame
        vImuMeas.clear();
        vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc.x,acc.y,acc.z,gyro.x,gyro.y,gyro.z,realsense.getGyroTimestamp()));

        rs2_time_t IR_timestamp = realsense.getIRLeftTimestamp();
        while(realsense.getGyroTimestamp() <= IR_timestamp)
        {
          // cout.precision(17);
          // cout << realsense.getGyroTimestamp() << " " << IR_timestamp << endl;
          acc  = realsense.getAccFrames();
          gyro = realsense.getGyroFrames();
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc.x,acc.y,acc.z,gyro.x,gyro.y,gyro.z,realsense.getGyroTimestamp()));
          realsense.updateIMU();
        }
        cout << vImuMeas.size() << endl;

        // vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc.x,acc.y,acc.z,gyro.x,gyro.y,gyro.z,realsense.getAccTimestamp()));

        // Pass the IR Left and Depth images to the SLAM system
        cv::Mat cameraPose = SLAM.TrackStereo(irLeftMatrix, irRightMatrix, realsense.getIRLeftTimestamp(), vImuMeas);

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




// #include <iostream>
// #include <chrono>
// #include <librealsense2/rs.hpp>
// using namespace std::chrono;
// using namespace std;

// typedef std::chrono::duration<float, std::ratio<1, 1>> seconds_f;

// bool check_imu_is_supported()
// {
//     bool found_gyro = false;
//     bool found_accel = false;
//     rs2::context ctx;
//     for (auto dev : ctx.query_devices())
//     {
//         // The same device should support gyro and accel
//         found_gyro = false;
//         found_accel = false;
//         for (auto sensor : dev.query_sensors())
//         {
//             for (auto profile : sensor.get_stream_profiles())
//             {
//                 if (profile.stream_type() == RS2_STREAM_GYRO)
//                     found_gyro = true;

//                 if (profile.stream_type() == RS2_STREAM_ACCEL)
//                     found_accel = true;
//             }
//         }
//         if (found_gyro && found_accel)
//             break;
//     }
//     return found_gyro && found_accel;
// }

// float get_frequency() {
//     static auto start = steady_clock::now();
//     auto end = steady_clock::now();
//     auto d = (seconds_f)(end - start);
//     start = end;
//     return 1.0f / d.count();
// }

// int main(int argc, char* argv[]) {
//     if (!check_imu_is_supported())
//     {
//         std::cerr << "Device supporting IMU (D435i) not found";
//         return EXIT_FAILURE;
//     }

//     // Declare RealSense pipeline, encapsulating the actual device and sensors
//     rs2::pipeline pipe;
//     // Create a configuration for configuring the pipeline with a non default profile
//     rs2::config cfg;

//     // Add streams of gyro and accelerometer to configuration
//     cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 250);
//     cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);

//     float fps = 0.0f;
//     int i = 0;
//     auto profile = pipe.start(cfg, [&](rs2::frame frame)
//         {
//             fps += 0.1*(get_frequency() - fps);
//             i++;
//             if (i % 10 == 0) {
//                 cout << fps << endl;
//             }
//             // Cast the frame that arrived to motion frame
//             auto motion = frame.as<rs2::motion_frame>();
//         });

//     getchar();
//     return EXIT_SUCCESS;
// }