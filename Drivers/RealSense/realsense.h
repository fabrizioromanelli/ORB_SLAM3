#ifndef __REALSENSE__
#define __REALSENSE__

#include <mutex>
#include <stack>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

class RealSense
{
public:
  // These enums are for setting the RealSense modalities.
  // RGBD - Uses RGB camera and Depth camera (not aligned, not synchronized)
  // IRD  - Uses Infrared Left camera and Depth camera (aligned, synchronized)
  // IRL  - Uses Infrared Left camera
  // IRR  - Uses Infrared Right camera
  // IMU_IRD  - Uses Infrared Left camera and Depth camera (aligned, synchronized) and IMU
  // IR_STEREO - Uses Infrared left and right camera
  // IMU_IR_STEREO - Uses Infrared left and right camera and IMU
  enum sModality { RGBD, IRD, IRL, IRR, IMU_IRD, IR_STEREO, IMU_IR_STEREO, FE_STEREO };

private:
  // Sensor modality
  sModality sensorModality;

  // Sensor configuration
  rs2::config config;

  // RealSense
  rs2::pipeline pipeline;
  rs2::pipeline_profile pipeline_profile;
  rs2::frameset aligned_frameset;
  rs2::device realSense_device;

  // Color Buffer
  rs2::frame color_frame;
  cv::Mat color_mat;
  uint32_t color_width = 640;
  uint32_t color_height = 480;
  uint32_t color_fps;

  // Fisheye Left Buffer
  rs2::frame fe_left_frame;
  cv::Mat fe_left_mat;
  uint32_t fe_left_width = 848;
  uint32_t fe_left_height = 800;
  uint32_t fe_left_fps;
  double fe_left_ts;

  // Fisheye Right Buffer
  rs2::frame fe_right_frame;
  cv::Mat fe_right_mat;
  uint32_t fe_right_width = 848;
  uint32_t fe_right_height = 800;
  uint32_t fe_right_fps;
  double fe_right_ts;

  // Infrared Left Buffer
  rs2::frame ir_left_frame;
  cv::Mat ir_left_mat;
  uint32_t ir_left_width = 640;
  uint32_t ir_left_height = 480;
  uint32_t ir_left_fps;
  double ir_left_ts;

  // Infrared Right Buffer
  rs2::frame ir_right_frame;
  cv::Mat ir_right_mat;
  uint32_t ir_right_width = 640;
  uint32_t ir_right_height = 480;
  uint32_t ir_right_fps;
  double ir_right_ts;

  // Depth Buffer
  rs2::frame depth_frame;
  cv::Mat depth_mat;
  uint32_t depth_width = 640;
  uint32_t depth_height = 480;
  uint32_t depth_fps;
  double depth_ts;

  // Gyro Buffer
  rs2::frame gyro_frame;
  rs2_vector gyro_data;
  uint32_t gyro_fps;
  std::vector<std::vector<double>> gyro_measurements;

  // Accelerometer Buffer
  rs2::frame acc_frame;
  rs2_vector acc_data;
  uint32_t acc_fps;
  std::vector<std::vector<double>> acc_measurements;

  // Warmup frames
  uint32_t warm_up_frames = 30;

  // Frameset
  rs2::frameset frameset;

  // Error
  rs2_error * e = 0;

  // Maximum delta between RGB and Depth image timeframes (time in ms)
  rs2_time_t maxDeltaTimeframes;
  rs2_time_t MIN_DELTA_TIMEFRAMES_THRESHOLD = 20;

  enum irCamera { IR_LEFT = 1, IR_RIGHT = 2};
  enum feCamera { FE_LEFT = 1, FE_RIGHT = 2};

  // Mutex for frame grabbing
  std::mutex accMtx, gyroMtx, frameMtx;

  // Stacks for the motion frames
  std::stack<std::vector<double>> accStack, gyroStack;

public:
  // Constructor
  RealSense(const sModality);

  // Constructor with maximum delta timeframes as an input
  RealSense(const sModality, double);

  // Constructor with fps as an input
  RealSense(const sModality, uint32_t);

  // Destructor
  ~RealSense();

  // Process
  void run();
  void getMotionFrequency();
  void startGrab();
  bool getFrames(cv::Mat &, cv::Mat &, std::vector<std::vector<double>> &, std::vector<std::vector<double>> &, double &);

  // Updates IMU frames
  void updateIMU();

  // Operations with frame timestamps
  rs2_time_t getRGBTimestamp();
  rs2_time_t getDepthTimestamp();
  rs2_time_t getIRLeftTimestamp();
  rs2_time_t getFETimestamp();
  rs2_time_t getTemporalFrameDisplacement();
  rs2_time_t getAverageTimestamp();
  rs2_time_t getGyroTimestamp();
  rs2_time_t getAccTimestamp();

  bool isValidAlignedFrame();

  // Get frame matrices
  cv::Mat getColorMatrix();
  cv::Mat getDepthMatrix();
  cv::Mat getIRLeftMatrix();
  cv::Mat getIRRightMatrix();
  cv::Mat getFELeftMatrix();
  cv::Mat getFERightMatrix();

  // Get raw frames
  rs2::frame getColorFrame();
  rs2::frame getDepthFrame();
  rs2::frame getIRLeftFrame();
  rs2::frame getIRRightFrame();
  rs2::frame getFELeftFrame();
  rs2::frame getFERightFrame();

  // Get IMU frames
  cv::Point3f getAccFrames();
  cv::Point3f getGyroFrames();

  // Control laser projector
  void enableLaser(float);
  void disableLaser();

private:
  // Initialize
  void initialize(rs2_time_t);

  // Initialize Sensor
  inline void initializeSensor();

  // Finalize
  void finalize();

  // Updates for aligned RGBD frames
  // Update Data
  void updateRGBD();

  // Updates for IRD frames
  void updateIRD();

  // Updates for IMU_IRD frame
  void updateIMU_IRD();

  // Updates for IR Left and Right frames
  void updateIRL();
  void updateIRR();

  // Updates for IR_STEREO frames
  void updateIR_STEREO();

  // Updates for IMU_IR_STEREO frames
  void updateIMU_IR_STEREO();

  // Updates for FE_STEREO frames
  void updateFE_STEREO();

  // Updates for motion frame
  void updateGyro();
  void updateAcc();

  // Update Frame
  inline void updateFrame();

  // Update Color
  inline void updateColor();

  // Update Depth
  inline void updateDepth();

  // Update IR (Left)
  inline void updateInfraredIRLeft();

  // Update IR (Right)
  inline void updateInfraredIRRight();

  // Update FE (Left)
  inline void updateFELeft();

  // Update FE (Right)
  inline void updateFERight();

  // Draw Data
  void draw();

  // Draw Color
  inline void drawColor();

  // Draw Depth
  inline void drawDepth();

  // Show Data
  void show();

  // Show Color
  inline void showColor();

  // Show Depth
  inline void showDepth();
};

#endif // __REALSENSE__