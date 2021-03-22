#include "realsense.h"

// Constructor
RealSense::RealSense(const sModality modality):
sensorModality(modality), color_fps(30), ir_left_fps(30), ir_right_fps(30), depth_fps(30), acc_fps(250), gyro_fps(400)
{
  initialize(MIN_DELTA_TIMEFRAMES_THRESHOLD);
}

// Constructor with maximum delta timeframes as an input
RealSense::RealSense(const sModality modality, double maximumDeltaTimeframes):
sensorModality(modality), color_fps(30), ir_left_fps(30), ir_right_fps(30), depth_fps(30), acc_fps(250), gyro_fps(400)
{
  if (maximumDeltaTimeframes > MIN_DELTA_TIMEFRAMES_THRESHOLD)
    initialize(maximumDeltaTimeframes);
  else
  {
    std::cerr << "Maximum delta between timeframes is too high. Reduced to " << MIN_DELTA_TIMEFRAMES_THRESHOLD << "." << std::endl;
    initialize(MIN_DELTA_TIMEFRAMES_THRESHOLD);
  }
}

RealSense::RealSense(const sModality modality, uint32_t fps):
sensorModality(modality), color_fps(fps), ir_left_fps(fps), ir_right_fps(fps), depth_fps(fps)
{
  initialize(MIN_DELTA_TIMEFRAMES_THRESHOLD);
}

// Destructor
RealSense::~RealSense()
{
  finalize();
}

// Public function for processing and updating sensor
void RealSense::run()
{
  switch (sensorModality)
  {
    case RGBD:
      updateRGBD();
      break;
    case IRD:
      updateIRD();
      break;
    case IRL:
      updateIRL();
      break;
    case IRR:
      updateIRR();
      break;
    case IMU_IRD:
      updateIMU_IRD();
      break;
    case IR_STEREO:
      updateIR_STEREO();
      break;
    case IMU_IR_STEREO:
      updateIMU_IR_STEREO();
      break;
    default:
      break;
  }
}

bool RealSense::getFrames(cv::Mat & irAMatrix, cv::Mat & irBMatrix, std::vector<std::vector<double>> & gyro, std::vector<std::vector<double>> & acc, double & vFrameTs)
{
  frameMtx.lock();
  if (!gyro_measurements.empty() && !acc_measurements.empty())
  {
    if (sensorModality == IMU_IR_STEREO)
    {
      irAMatrix = this->getIRLeftMatrix();
      irBMatrix = this->getIRRightMatrix();
    }
    else if (sensorModality == IMU_IRD)
    {
      irAMatrix = this->getIRLeftMatrix();
      irBMatrix = this->getDepthMatrix();
    }

    gyro      = gyro_measurements;
    acc       = acc_measurements;
    vFrameTs  = ir_left_ts;
    frameMtx.unlock();
    return(true);
  }
  else
  {
    frameMtx.unlock();
    return(false);
  }
}

void RealSense::startGrab()
{
  // std::cout.precision(17);
  pipeline.stop();
  auto profile = pipeline.start(config, [&](rs2::frame frame)
    {
      // Is a video frame
      if (frame.is<rs2::frameset>())
      {
        frameMtx.lock();
        auto fs        = frame.as<rs2::frameset>();

        if (sensorModality == IMU_IR_STEREO)
        {
          auto irLFrame  = fs.get_infrared_frame(IR_LEFT);
          auto irRFrame  = fs.get_infrared_frame(IR_RIGHT);
          ir_left_frame  = irLFrame;
          ir_left_ts     = irLFrame.get_timestamp()/1000.0;
          ir_right_frame = irRFrame;
          ir_right_ts    = irRFrame.get_timestamp()/1000.0;
        }
        else if (sensorModality == IMU_IRD)
        {
          auto irLFrame  = fs.get_infrared_frame(IR_LEFT);
          auto dFrame    = fs.get_depth_frame();
          ir_left_frame  = irLFrame;
          ir_left_ts     = irLFrame.get_timestamp()/1000.0;
          depth_frame    = dFrame;
          depth_ts       = dFrame.get_timestamp()/1000.0;
        }

        accMtx.lock();
        gyroMtx.lock();
        // Loop over acc stack as accelerometer is slower than gyroscope
        for (size_t i = 0; i < accStack.size(); i++)
        {
          if (accStack.top()[0] > ir_left_ts)
            accStack.pop();

          if (gyroStack.top()[0] > ir_left_ts)
            gyroStack.pop();
        }

        gyro_measurements.clear();
        acc_measurements.clear();

        size_t accSize  = accStack.size();
        size_t gyroSize = gyroStack.size();
        for (size_t i = 0; i < std::min(accSize, gyroSize); i++)
        {
          // TODO align as best as possibile the acc and gyro stacks
          gyro_measurements.push_back(gyroStack.top());
          gyroStack.pop();
          acc_measurements.push_back(accStack.top());
          accStack.pop();
        }

        // Empty the accelerometer and gyroscope stacks
        accStack  = std::stack<std::vector<double>>();
        gyroStack = std::stack<std::vector<double>>();

        accMtx.unlock();
        gyroMtx.unlock();
        frameMtx.unlock();
      }
      // Is a motion frame
      else if (frame.is<rs2::motion_frame>())
      {
        auto motion = frame.as<rs2::motion_frame>();
        if (motion.get_profile().stream_type() == rs2_stream::RS2_STREAM_ACCEL)
        { 
          accMtx.lock();
          std::vector<double> tmp;
          tmp.push_back(motion.get_timestamp()/1000.0);
          tmp.push_back(motion.get_motion_data().x);
          tmp.push_back(motion.get_motion_data().y);
          tmp.push_back(motion.get_motion_data().z);
          accStack.push(tmp);
          accMtx.unlock();
        }
        else if (motion.get_profile().stream_type() == rs2_stream::RS2_STREAM_GYRO)
        {
          gyroMtx.lock();
          std::vector<double> tmp;
          tmp.push_back(motion.get_timestamp()/1000.0);
          tmp.push_back(motion.get_motion_data().x);
          tmp.push_back(motion.get_motion_data().y);
          tmp.push_back(motion.get_motion_data().z);
          gyroStack.push(tmp);
          gyroMtx.unlock();
        }
      }
    });
}

void RealSense::getMotionFrequency()
{
  static int accCount = 0, gyroCount = 0;
  static double prevAccTs = 0.0, prevGyroTs = 0.0;
  std::cout.precision(17);
  pipeline.stop();
  auto profile = pipeline.start(config, [&](rs2::frame frame)
    {
      // Cast the frame that arrived to motion frame
      auto motion = frame.as<rs2::motion_frame>();
      if (motion.get_profile().stream_type() == rs2_stream::RS2_STREAM_ACCEL)
      {
        double currentTs = trunc(motion.get_timestamp()/1000.0);
        if (currentTs - prevAccTs < 1.0)
          accCount++;
        else
        {
          std::cout << "ACC freq:  " << accCount << "Hz" << std::endl;
          accCount = 0;
          prevAccTs = trunc(motion.get_timestamp()/1000.0);
        }
      }
      else if (motion.get_profile().stream_type() == rs2_stream::RS2_STREAM_GYRO)
      {
        double currentTs = trunc(motion.get_timestamp()/1000.0);
        if (currentTs - prevGyroTs < 1.0)
        {
          gyroCount++;
        }
        else
        {
          std::cout << "GYRO freq: " << gyroCount << "Hz" << std::endl;
          gyroCount = 0;
          prevGyroTs = trunc(motion.get_timestamp()/1000.0);
        }
      }
    });
}

rs2_time_t RealSense::getRGBTimestamp()
{
  if (sensorModality == RGBD) {
    // Get each frame
    rs2::frame cFrame  = aligned_frameset.get_color_frame();
    rs2_frame * frameP = cFrame.get();

    // Get frame timestamp
    return(rs2_get_frame_timestamp(frameP, &e)/1000.0);
  } else {
    return(-1);
  }
}

rs2_time_t RealSense::getDepthTimestamp()
{
  rs2::frame cFrame;
  switch (sensorModality)
  {
    case RGBD:
      cFrame  = aligned_frameset.get_depth_frame();
      break;
    case IRD:
    case IMU_IRD:
      cFrame  = frameset.get_depth_frame();
      break;
    default:
      std::cerr << "NOT IMPLEMENTED" << std::endl;
      break;
  }

  rs2_frame * frameP = cFrame.get();

  // Get frame timestamp
  return(rs2_get_frame_timestamp(frameP, &e)/1000.0);
}

rs2_time_t RealSense::getIRLeftTimestamp()
{
  if ((sensorModality == IMU_IRD) || (sensorModality == IRD) || (sensorModality == IRL) || (sensorModality == IR_STEREO) || (sensorModality == IMU_IR_STEREO)) {
    // Get each frame
    rs2::frame cFrame  = frameset.get_infrared_frame(IR_LEFT);
    rs2_frame * frameP = cFrame.get();

    // Get frame timestamp
    return(rs2_get_frame_timestamp(frameP, &e)/1000.0);
  } else {
    return(-1);
  }
}

rs2_time_t RealSense::getGyroTimestamp()
{
  gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
  if (gyro_frame)
  {
    rs2_frame * frameP = gyro_frame.get();
    return(rs2_get_frame_timestamp(frameP, &e)/1000.0);
  } else {
    return(-1);
  }
}

rs2_time_t RealSense::getAccTimestamp()
{
  acc_frame = frameset.first_or_default(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
  if (acc_frame)
  {
    rs2_frame * frameP = acc_frame.get();
    return(rs2_get_frame_timestamp(frameP, &e)/1000.0);
  } else {
    return(-1);
  }
}


// This function gets the temporal displacement between
// the RGB and Depth frames (their delta).
rs2_time_t RealSense::getTemporalFrameDisplacement()
{
  switch (sensorModality)
  {
    case RGBD:
      return(fabs(getRGBTimestamp()-getDepthTimestamp())/1000.0);
      break;
    case IRD:
    case IMU_IRD:
      return(fabs(getIRLeftTimestamp()-getDepthTimestamp())/1000.0);
      break;
    default:
      std::cerr << "NOT IMPLEMENTED" << std::endl;
      break;
  }
  return(0);
}

// This function gets the average timestamp between
// the RGB and Depth frames.
rs2_time_t RealSense::getAverageTimestamp()
{
  switch (sensorModality)
  {
    case RGBD:
      return(((getRGBTimestamp()+getDepthTimestamp())/2.0)/1000.0);
      break;
    case IRD:
    case IMU_IRD:
      return(((getIRLeftTimestamp()+getDepthTimestamp())/2.0)/1000.0);
      break;
    default:
      std::cerr << "NOT IMPLEMENTED" << std::endl;
      break;
  }
  return(0);
}

bool RealSense::isValidAlignedFrame()
{
  if (sensorModality == RGBD) {
    if (getTemporalFrameDisplacement() >= maxDeltaTimeframes)
      return(false);
    else
      return(true);
  } else {
    return(true);
  }
}

// Get color matrix
cv::Mat RealSense::getColorMatrix()
{
  cv::Mat color(cv::Size(color_width, color_height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
  return(color);
}

// Get depth matrix
cv::Mat RealSense::getDepthMatrix()
{
  cv::Mat depth(cv::Size(depth_width, depth_height), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
  return(depth);
}

// Get IR left matrix
cv::Mat RealSense::getIRLeftMatrix()
{
  cv::Mat ir_left(cv::Size(ir_left_width, ir_left_height), CV_8UC1, (void*)ir_left_frame.get_data(), cv::Mat::AUTO_STEP);
  return(ir_left);
}

// Get IR right matrix
cv::Mat RealSense::getIRRightMatrix()
{
  cv::Mat ir_right(cv::Size(ir_right_width, ir_right_height), CV_8UC1, (void*)ir_right_frame.get_data(), cv::Mat::AUTO_STEP);
  return(ir_right);
}

// Get color frame
rs2::frame RealSense::getColorFrame()
{
  return(color_frame);
}

// Get depth frame
rs2::frame RealSense::getDepthFrame()
{
  return(depth_frame);
}

// Get IR left frame
rs2::frame RealSense::getIRLeftFrame()
{
  return(ir_left_frame);
}

// Get IR right frame
rs2::frame RealSense::getIRRightFrame()
{
  return(ir_right_frame);
}

cv::Point3f RealSense::getGyroFrames()
{
  cv::Point3f gyro;
  gyro.x = gyro_data.x;
  gyro.y = gyro_data.y;
  gyro.z = gyro_data.z;
  return(gyro);
}

cv::Point3f RealSense::getAccFrames()
{
  cv::Point3f acc;
  acc.x = acc_data.x;
  acc.y = acc_data.y;
  acc.z = acc_data.z;
  return(acc);
}

// Initialize
void RealSense::initialize(rs2_time_t _maxDeltaTimeFrames)
{
  maxDeltaTimeframes = _maxDeltaTimeFrames;
  cv::setUseOptimized(true);
  initializeSensor();
}

// Initialize Sensor
inline void RealSense::initializeSensor()
{
  switch (sensorModality)
  {
    case RGBD:
      config.enable_stream( rs2_stream::RS2_STREAM_COLOR, color_width, color_height, rs2_format::RS2_FORMAT_BGR8, color_fps );
      config.enable_stream( rs2_stream::RS2_STREAM_DEPTH, depth_width, depth_height, rs2_format::RS2_FORMAT_Z16, depth_fps );
      break;
    case IRD:
      config.enable_stream( rs2_stream::RS2_STREAM_INFRARED, IR_LEFT, ir_left_width, ir_left_height, rs2_format::RS2_FORMAT_Y8, ir_left_fps );
      // The following is just needed to get the correct baseline information, it will be then disabled.
      config.enable_stream( rs2_stream::RS2_STREAM_INFRARED, IR_RIGHT, ir_right_width, ir_right_height, rs2_format::RS2_FORMAT_Y8, ir_right_fps );
      config.enable_stream( rs2_stream::RS2_STREAM_DEPTH, depth_width, depth_height, rs2_format::RS2_FORMAT_Z16, depth_fps );
      break;
    case IRL:
      config.enable_stream( rs2_stream::RS2_STREAM_INFRARED, IR_LEFT, ir_left_width, ir_left_height, rs2_format::RS2_FORMAT_Y8, ir_left_fps );
      break;
    case IRR:
      config.enable_stream( rs2_stream::RS2_STREAM_INFRARED, IR_RIGHT, ir_right_width, ir_right_height, rs2_format::RS2_FORMAT_Y8, ir_right_fps );
      break;
    case IMU_IRD:
      config.enable_stream( rs2_stream::RS2_STREAM_INFRARED, IR_LEFT, ir_left_width, ir_left_height, rs2_format::RS2_FORMAT_Y8, ir_left_fps );
      // The following is just needed to get the correct baseline information, it will be then disabled.
      config.enable_stream( rs2_stream::RS2_STREAM_INFRARED, IR_RIGHT, ir_right_width, ir_right_height, rs2_format::RS2_FORMAT_Y8, ir_right_fps );
      config.enable_stream( rs2_stream::RS2_STREAM_DEPTH, depth_width, depth_height, rs2_format::RS2_FORMAT_Z16, depth_fps );
      // Add streams of gyro and accelerometer to configuration
      config.enable_stream( rs2_stream::RS2_STREAM_ACCEL, rs2_format::RS2_FORMAT_MOTION_XYZ32F, acc_fps );
      config.enable_stream( rs2_stream::RS2_STREAM_GYRO, rs2_format::RS2_FORMAT_MOTION_XYZ32F, gyro_fps );
      break;
    case IR_STEREO:
      config.enable_stream( rs2_stream::RS2_STREAM_INFRARED, IR_LEFT, ir_left_width, ir_left_height, rs2_format::RS2_FORMAT_Y8, ir_left_fps );
      config.enable_stream( rs2_stream::RS2_STREAM_INFRARED, IR_RIGHT, ir_right_width, ir_right_height, rs2_format::RS2_FORMAT_Y8, ir_right_fps );
      break;
    case IMU_IR_STEREO:
      config.enable_stream( rs2_stream::RS2_STREAM_INFRARED, IR_LEFT, ir_left_width, ir_left_height, rs2_format::RS2_FORMAT_Y8, ir_left_fps );
      config.enable_stream( rs2_stream::RS2_STREAM_INFRARED, IR_RIGHT, ir_right_width, ir_right_height, rs2_format::RS2_FORMAT_Y8, ir_right_fps );
      // Add streams of gyro and accelerometer to configuration
      config.enable_stream( rs2_stream::RS2_STREAM_ACCEL, rs2_format::RS2_FORMAT_MOTION_XYZ32F, acc_fps );
      config.enable_stream( rs2_stream::RS2_STREAM_GYRO,  rs2_format::RS2_FORMAT_MOTION_XYZ32F, gyro_fps );
      break;
    default:
      std::cerr << "Invalid modality selected" << std::endl;
      break;
  }

  pipeline_profile = pipeline.start(config);
  realSense_device = pipeline_profile.get_device();

  // Refer to: https://github.com/raulmur/ORB_SLAM2/issues/259
  if ((sensorModality == IRD) || (sensorModality == IMU_IRD))
  {
    auto depth_sensor = realSense_device.first<rs2::depth_sensor>();
    auto depth_stream = pipeline_profile.get_stream(RS2_STREAM_DEPTH);
    auto ir_stream    = pipeline_profile.get_stream(RS2_STREAM_INFRARED, 2);

    const float scale = depth_sensor.get_depth_scale();
    rs2_intrinsics intrinsics = pipeline_profile.get_stream(RS2_STREAM_INFRARED).as<rs2::video_stream_profile>().get_intrinsics();
    rs2_extrinsics extrinsics = depth_stream.get_extrinsics_to(ir_stream);

    std::stringstream ss;
    ss << "    " << std::left << std::setw(31) << "Width"      << ": " << intrinsics.width << std::endl <<
          "    " << std::left << std::setw(31) << "Height"     << ": " << intrinsics.height << std::endl <<
          "    " << std::left << std::setw(31) << "Distortion" << ": " << rs2_distortion_to_string(intrinsics.model) << std::endl <<
          "    " << std::left << std::setw(31) << "Baseline"   << ": " << std::setprecision(15) << extrinsics.translation[0] << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.fx"  << ": " << std::setprecision(15) << intrinsics.fx << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.fy"  << ": " << std::setprecision(15) << intrinsics.fy << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.cx"  << ": " << std::setprecision(15) << intrinsics.ppx << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.cy"  << ": " << std::setprecision(15) << intrinsics.ppy << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.k1"  << ": " << std::setprecision(15) << intrinsics.coeffs[0] << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.k2"  << ": " << std::setprecision(15) << intrinsics.coeffs[1] << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.p1"  << ": " << std::setprecision(15) << intrinsics.coeffs[2] << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.p1"  << ": " << std::setprecision(15) << intrinsics.coeffs[3] << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.k3"  << ": " << std::setprecision(15) << intrinsics.coeffs[4] << std::endl <<
          "    " << std::left << std::setw(31) << "DepthMapFactor" << ": " << 1/scale << std::endl <<
          "    " << std::left << std::setw(31) << "Camera.bf"  << ": " << fabs(extrinsics.translation[0]*intrinsics.fx) << std::endl;

    std::cout << ss.str() << std::endl;
    pipeline.stop();
    config.disable_stream(RS2_STREAM_INFRARED, 2);
    pipeline_profile = pipeline.start(config);
    realSense_device = pipeline_profile.get_device();
  }

  // The laser projector is disabled by default
  disableLaser();

  // Camera warmup - dropping several first frames to let auto-exposure stabilize
  for (uint32_t i = 0; i < warm_up_frames; i++)
  {
    // Wait for all configured streams to produce a frame
    frameset = pipeline.wait_for_frames();
  }
}

void RealSense::enableLaser(float power)
{
  auto depth_sensor = realSense_device.first<rs2::depth_sensor>();

  if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    depth_sensor.set_option(RS2_OPTION_LASER_POWER, power);
}

void RealSense::disableLaser()
{
  auto depth_sensor = realSense_device.first<rs2::depth_sensor>();

  if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
    depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
}

// Finalize
void RealSense::finalize()
{
  cv::destroyAllWindows();
  pipeline.stop();
}

// Update Data
void RealSense::updateRGBD()
{
  updateFrame();
  updateColor();
  updateDepth();
}

void RealSense::updateIRD()
{
  updateFrame();
  updateInfraredIRLeft();
  updateDepth();
}

void RealSense::updateIMU_IRD()
{
  updateFrame();
  updateInfraredIRLeft();
  updateDepth();
  updateGyro();
  updateAcc();
}

void RealSense::updateIRL()
{
  updateFrame();
  updateInfraredIRLeft();
}

void RealSense::updateIRR()
{
  updateFrame();
  updateInfraredIRRight();
}

void RealSense::updateIR_STEREO()
{
  updateFrame();
  updateInfraredIRLeft();
  updateInfraredIRRight();
}

void RealSense::updateIMU_IR_STEREO()
{
  updateFrame();
  updateInfraredIRLeft();
  updateInfraredIRRight();
  updateGyro();
  updateAcc();
}

void RealSense::updateIMU()
{
  updateFrame();
  updateGyro();
  updateAcc();
}

// Update Frame
inline void RealSense::updateFrame()
{
  frameset = pipeline.wait_for_frames();

  if (sensorModality == RGBD) {
    // Retrieve Aligned Frame
    rs2::align align( rs2_stream::RS2_STREAM_COLOR );
    aligned_frameset = align.process( frameset );
    if( !aligned_frameset.size() ){
      return;
    }
  }
}

// Update Color
inline void RealSense::updateColor()
{
  if (sensorModality == RGBD)
    color_frame = aligned_frameset.get_color_frame();
  else
    color_frame = frameset.get_color_frame();

  // Retrieve Frame Information
  color_width = color_frame.as<rs2::video_frame>().get_width();
  color_height = color_frame.as<rs2::video_frame>().get_height();
}

// Update Depth
inline void RealSense::updateDepth()
{
  if (sensorModality == RGBD)
    depth_frame = aligned_frameset.get_depth_frame();
  else
    depth_frame = frameset.get_depth_frame();

  // Retrieve Frame Information
  depth_width = depth_frame.as<rs2::video_frame>().get_width();
  depth_height = depth_frame.as<rs2::video_frame>().get_height();
}

// Update Infrared (Left)
inline void RealSense::updateInfraredIRLeft()
{
  ir_left_frame  = frameset.get_infrared_frame(IR_LEFT);

  // Retrieve Frame Information
  ir_left_width  = ir_left_frame.as<rs2::video_frame>().get_width();
  ir_left_height = ir_left_frame.as<rs2::video_frame>().get_height();
}

// Update Infrared (Right)
inline void RealSense::updateInfraredIRRight()
{
  ir_right_frame  = frameset.get_infrared_frame(IR_RIGHT);

  // Retrieve Frame Information
  ir_right_width  = ir_right_frame.as<rs2::video_frame>().get_width();
  ir_right_height = ir_right_frame.as<rs2::video_frame>().get_height();
}

// Update Gyro
inline void RealSense::updateGyro()
{
  gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
  if (gyro_frame)
  {
    auto motion = gyro_frame.as<rs2::motion_frame>();

    // Retrieve Frame Information
    gyro_data = motion.get_motion_data();
  }
}

// Update Accelerometer
inline void RealSense::updateAcc()
{
  acc_frame = frameset.first_or_default(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
  if (acc_frame)
  {
    auto motion = acc_frame.as<rs2::motion_frame>();

    // Retrieve Frame Information
    acc_data = motion.get_motion_data();
  }
}

// Draw Data
void RealSense::draw()
{
  drawColor();
  drawDepth();
}

// Draw Color
inline void RealSense::drawColor()
{
  color_mat = cv::Mat( color_height, color_width, CV_8UC3, const_cast<void*>( color_frame.get_data() ) );
}

// Draw Depth
inline void RealSense::drawDepth()
{
  depth_mat = cv::Mat( depth_height, depth_width, CV_16SC1, const_cast<void*>( depth_frame.get_data() ) );
}

// Show Data
void RealSense::show()
{
  showColor();
  showDepth();
}

// Show Color
inline void RealSense::showColor()
{
  if( color_mat.empty() ){
    return;
  }

  cv::imshow( "Color", color_mat );
}

// Show Depth
inline void RealSense::showDepth()
{
  if( depth_mat.empty() ){
    return;
  }

  // Scaling
  cv::Mat scale_mat;
  depth_mat.convertTo( scale_mat, CV_8U, -255.0 / 10000.0, 255.0 ); // 0-10000 -> 255(white)-0(black)
  //depth_mat.convertTo( scale_mat, CV_8U, 255.0 / 10000.0, 0.0 ); // 0-10000 -> 0(black)-255(white)

  cv::imshow( "Depth", scale_mat );
}
