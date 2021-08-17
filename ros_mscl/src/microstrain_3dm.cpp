/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Parker-Lord Inertial Device Driver Implementation File
//
// Copyright (c) 2017, Brian Bingham
// Copyright (c) 2020, Parker Hannifin Corp
// This code is licensed under MIT license (see LICENSE file for details)
//
/////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Include Files
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <algorithm>
#include <time.h>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include <ctime>

#include "mscl/mscl.h"
#include "microstrain_diagnostic_updater.h"
#include <ros/callback_queue.h>
#include <tf2/LinearMath/Transform.h>

#include <sys/stat.h>


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization
/////////////////////////////////////////////////////////////////////////////////////////////////////

namespace Microstrain
{

Microstrain::Microstrain()
{
  m_use_device_timestamp = false;
  m_com_mode = 0;
  m_filter_valid_packet_count = 0;
  m_imu_valid_packet_count= 0;
  m_gnss_valid_packet_count[GNSS1_ID]= 0;
  m_gnss_valid_packet_count[GNSS2_ID]= 0;
  m_filter_timeout_packet_count= 0;
  m_imu_timeout_packet_count= 0;
  m_gnss_timeout_packet_count[GNSS1_ID]= 0;
  m_gnss_timeout_packet_count[GNSS2_ID]= 0;
  m_filter_checksum_error_packet_count= 0;
  m_imu_checksum_error_packet_count= 0;
  m_gnss_checksum_error_packet_count[GNSS1_ID]= 0;
  m_gnss_checksum_error_packet_count[GNSS2_ID]= 0;
  m_imu_frame_id = "sensor";
  m_gnss_frame_id[GNSS1_ID] = "gnss1_antenna_wgs84_ned";
  m_gnss_frame_id[GNSS2_ID] = "gnss2_antenna_wgs84_ned";
  m_filter_frame_id = "sensor_wgs84_ned";
  m_filter_child_frame_id = "sensor";
  m_publish_imu = true;
  m_publish_gps_corr = false;
  m_gps_leap_seconds = 18.0;
  m_publish_gnss[GNSS1_ID]= true;
  m_publish_gnss[GNSS2_ID]= true;
  m_publish_gnss_aiding_status[GNSS1_ID] = false;
  m_publish_gnss_aiding_status[GNSS2_ID] = false;
  m_publish_gnss_dual_antenna_status = false;
  m_publish_filter = true;
  m_publish_rtk = false;
  m_imu_linear_cov = std::vector<double>(9, 0.0);
  m_imu_angular_cov = std::vector<double>(9, 0.0);
  m_imu_orientation_cov = std::vector<double>(9, 0.0);
  m_raw_file_enable = false;
  m_raw_file_include_support_data = false;

  m_t_ned2enu = tf2::Matrix3x3(0,1,0,1,0,0,0,0,-1);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Run Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Microstrain::run()
{
  // Variables for device configuration, ROS parameters, etc.
  uint32_t com_port;
  bool     device_setup      = false;
  bool     save_settings     = true;
  bool     filter_auto_init  = true;
  int      declination_source;
  double   declination;
  uint8_t  readback_declination_source;
  int      heading_source;
  float    initial_heading;
  uint16_t duration = 0;
  std::vector<double> default_matrix(9, 0.0);
  std::vector<double> default_vector(3, 0.0);
  std::vector<double> default_quaternion(4, 0.0);
  int dynamics_mode;
  bool filter_reset_after_config;
  int  filter_sensor2vehicle_frame_selector;
  std::vector<double> filter_sensor2vehicle_frame_transformation_euler(3, 0.0);
  std::vector<double> filter_sensor2vehicle_frame_transformation_matrix(9, 0.0);
  std::vector<double> filter_sensor2vehicle_frame_transformation_quaternion(4, 0.0);

  //RTK Dongle
  bool rtk_dongle_enable = false;

  //GQ7 Filter options
  int filter_adaptive_level;
  int filter_adaptive_time_limit_ms;
  bool filter_enable_gnss_pos_vel_aiding;
  bool filter_enable_gnss_heading_aiding;
  bool filter_enable_altimeter_aiding;
  bool filter_enable_odometer_aiding;
  bool filter_enable_magnetometer_aiding;
  bool filter_enable_external_heading_aiding;
  bool filter_enable_external_gps_time_update;
  int filter_enable_acceleration_constraint;
  int filter_enable_velocity_constraint;
  int filter_enable_angular_constraint;
  int filter_init_condition_src;
  int filter_auto_heading_alignment_selector;
  int filter_init_reference_frame;
  std::vector<double> filter_init_position(3, 0.0);
  std::vector<double> filter_init_velocity(3, 0.0);
  std::vector<double> filter_init_attitude(3, 0.0);
  int filter_relative_position_frame;
  std::vector<double> filter_relative_position_ref(3, 0.0);
  std::vector<double> filter_speed_lever_arm(3, 0.0);
  bool filter_enable_wheeled_vehicle_constraint;
  bool filter_enable_vertical_gyro_constraint;
  bool filter_enable_gnss_antenna_cal;
  double filter_gnss_antenna_cal_max_offset;
  int filter_pps_source;

  //GPIO Config options
  bool gpio_config;
  int gpio1_feature;
  int gpio1_behavior;
  int gpio1_pin_mode;
  int gpio2_feature;
  int gpio2_behavior;
  int gpio2_pin_mode;
  int gpio3_feature;
  int gpio3_behavior;
  int gpio3_pin_mode;
  int gpio4_feature;
  int gpio4_behavior;
  int gpio4_pin_mode;


  // ROS setup
  ros::Time::init();
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  // Comms Parameters
  std::string port;
  int baudrate;

  //Raw datafile
  std::string raw_file_directory;

  ///////////////////////////////////////////////////////////////////////////
  // Process the parameters
  ///////////////////////////////////////////////////////////////////////////

  //Initialize defaults

  //Matrix (eye)
  default_matrix[0] = default_matrix[4] = default_matrix[8] = 1.0;

  //Quat (unit)
  default_quaternion[3] = 1.0;


  // Configuration Parameters Load

  //Device
  private_nh.param("port",                  port, std::string("/dev/ttyACM1"));
  private_nh.param("baudrate",              baudrate, 115200);
  private_nh.param("device_setup",          device_setup, false);
  private_nh.param("save_settings",         save_settings, true);
  private_nh.param("use_device_timestamp",  m_use_device_timestamp, false);
  private_nh.param("use_enu_frame",         m_use_enu_frame, false);

  //If using ENU frame, reflect in the device frame id
  if(m_use_enu_frame)
  {
    m_gnss_frame_id[GNSS1_ID] = "gnss1_antenna_wgs84_enu";
    m_gnss_frame_id[GNSS2_ID] = "gnss2_antenna_wgs84_enu";
    m_filter_frame_id         = "sensor_wgs84_enu";
  }


  //IMU
  private_nh.param("publish_imu",           m_publish_imu, true);
  private_nh.param("publish_gps_corr",      m_publish_gps_corr, false);
  private_nh.param("imu_data_rate",         m_imu_data_rate, 10);
  private_nh.param("imu_orientation_cov",   m_imu_orientation_cov, default_matrix);
  private_nh.param("imu_linear_cov",        m_imu_linear_cov,      default_matrix);
  private_nh.param("imu_angular_cov",       m_imu_angular_cov,     default_matrix);
  private_nh.param("imu_frame_id",          m_imu_frame_id, m_imu_frame_id);

  //GNSS 1/2
  private_nh.param("publish_gnss1",         m_publish_gnss[GNSS1_ID], false);
  private_nh.param("publish_gnss2",         m_publish_gnss[GNSS2_ID], false);
  private_nh.param("gnss1_data_rate",       m_gnss_data_rate[GNSS1_ID], 1);
  private_nh.param("gnss2_data_rate",       m_gnss_data_rate[GNSS2_ID], 1);
  private_nh.param("gnss1_antenna_offset",  m_gnss_antenna_offset[GNSS1_ID],  default_vector);
  private_nh.param("gnss2_antenna_offset",  m_gnss_antenna_offset[GNSS2_ID],  default_vector);
  private_nh.param("gnss1_frame_id",        m_gnss_frame_id[GNSS1_ID], m_gnss_frame_id[GNSS1_ID]);
  private_nh.param("gnss2_frame_id",        m_gnss_frame_id[GNSS2_ID], m_gnss_frame_id[GNSS2_ID]);

  //RTK Dongle configuration
  private_nh.param("rtk_dongle_enable",  rtk_dongle_enable,  false);

  //Filter
  private_nh.param("publish_filter",             m_publish_filter, false);
  private_nh.param("filter_reset_after_config",  filter_reset_after_config, true);
  private_nh.param("filter_auto_init",           filter_auto_init, true);
  private_nh.param("filter_data_rate",           m_filter_data_rate, 10);
  private_nh.param("filter_frame_id",            m_filter_frame_id, m_filter_frame_id);
  private_nh.param("publish_relative_position",  m_publish_filter_relative_pos, false);
  private_nh.param("filter_sensor2vehicle_frame_selector",                  filter_sensor2vehicle_frame_selector, 0);
  private_nh.param("filter_sensor2vehicle_frame_transformation_euler",      filter_sensor2vehicle_frame_transformation_euler,      default_vector);
  private_nh.param("filter_sensor2vehicle_frame_transformation_matrix",     filter_sensor2vehicle_frame_transformation_matrix,     default_matrix);
  private_nh.param("filter_sensor2vehicle_frame_transformation_quaternion", filter_sensor2vehicle_frame_transformation_quaternion, default_quaternion);

  private_nh.param("filter_initial_heading",          initial_heading, (float)0.0);
  private_nh.param("filter_heading_source",           heading_source, 0x1);
  private_nh.param("filter_declination_source",       declination_source, 2);
  private_nh.param("filter_declination",              declination, 0.23);
  private_nh.param("filter_dynamics_mode",            dynamics_mode, 1);
  private_nh.param("filter_pps_source",               filter_pps_source, 1);
  private_nh.param("gps_leap_seconds",                m_gps_leap_seconds, 18.0);
  private_nh.param("filter_angular_zupt",             m_angular_zupt, false);
  private_nh.param("filter_velocity_zupt",            m_velocity_zupt, false);
  private_nh.param("filter_velocity_zupt_topic",      m_velocity_zupt_topic, std::string("/moving_vel"));
  private_nh.param("filter_angular_zupt_topic",       m_angular_zupt_topic, std::string("/moving_ang"));
  private_nh.param("filter_external_gps_time_topic",  m_external_gps_time_topic, std::string("/external_gps_time"));

  //Additional GQ7 Filter
  private_nh.param("filter_adaptive_level" ,                   filter_adaptive_level, 2);
  private_nh.param("filter_adaptive_time_limit_ms" ,           filter_adaptive_time_limit_ms, 15000);
  private_nh.param("filter_enable_gnss_pos_vel_aiding",        filter_enable_gnss_pos_vel_aiding, true);
  private_nh.param("filter_enable_gnss_heading_aiding",        filter_enable_gnss_heading_aiding, true);
  private_nh.param("filter_enable_altimeter_aiding",           filter_enable_altimeter_aiding, false);
  private_nh.param("filter_enable_odometer_aiding",            filter_enable_odometer_aiding, false);
  private_nh.param("filter_enable_magnetometer_aiding",        filter_enable_magnetometer_aiding, false);
  private_nh.param("filter_enable_external_heading_aiding",    filter_enable_external_heading_aiding, false);
  private_nh.param("filter_enable_external_gps_time_update",   filter_enable_external_gps_time_update, false);
  private_nh.param("filter_enable_acceleration_constraint",    filter_enable_acceleration_constraint, 0);
  private_nh.param("filter_enable_velocity_constraint",        filter_enable_velocity_constraint, 0);
  private_nh.param("filter_enable_angular_constraint",         filter_enable_angular_constraint, 0);
  private_nh.param("filter_init_condition_src",                filter_init_condition_src, 0);
  private_nh.param("filter_auto_heading_alignment_selector",   filter_auto_heading_alignment_selector, 0);
  private_nh.param("filter_init_reference_frame",              filter_init_reference_frame, 2);
  private_nh.param("filter_init_position",                     filter_init_position, default_vector);
  private_nh.param("filter_init_velocity",                     filter_init_velocity, default_vector);
  private_nh.param("filter_init_attitude",                     filter_init_attitude, default_vector);
  private_nh.param("filter_relative_position_frame",           filter_relative_position_frame, 2);
  private_nh.param("filter_relative_position_ref",             filter_relative_position_ref, default_vector);
  private_nh.param("filter_speed_lever_arm",                   filter_speed_lever_arm, default_vector);
  private_nh.param("filter_enable_wheeled_vehicle_constraint", filter_enable_wheeled_vehicle_constraint, false);
  private_nh.param("filter_enable_vertical_gyro_constraint",   filter_enable_vertical_gyro_constraint, false);
  private_nh.param("filter_enable_gnss_antenna_cal",           filter_enable_gnss_antenna_cal, false);
  private_nh.param("filter_gnss_antenna_cal_max_offset",       filter_gnss_antenna_cal_max_offset, 0.1);

  //GPIO Configuration
  private_nh.param("gpio1_feature",   gpio1_feature,  0);
  private_nh.param("gpio1_behavior",  gpio1_behavior, 0);
  private_nh.param("gpio1_pin_mode",  gpio1_pin_mode, 0);

  private_nh.param("gpio2_feature",   gpio2_feature,  0);
  private_nh.param("gpio2_behavior",  gpio2_behavior, 0);
  private_nh.param("gpio2_pin_mode",  gpio2_pin_mode, 0);

  private_nh.param("gpio3_feature",   gpio3_feature,  0);
  private_nh.param("gpio3_behavior",  gpio3_behavior, 0);
  private_nh.param("gpio3_pin_mode",  gpio3_pin_mode, 0);

  private_nh.param("gpio4_feature",   gpio4_feature,  0);
  private_nh.param("gpio4_behavior",  gpio4_behavior, 0);
  private_nh.param("gpio4_pin_mode",  gpio4_pin_mode, 0);

  private_nh.param("gpio_config",     gpio_config, false);

  //Raw data file save
  private_nh.param("raw_file_enable",               m_raw_file_enable, false);
  private_nh.param("raw_file_include_support_data", m_raw_file_include_support_data, false);
  private_nh.param("raw_file_directory",            raw_file_directory, std::string("."));

  ROS_INFO("Using MSCL Version: %s", mscl::MSCL_VERSION.str().c_str());


  ///////////////////////////////////////////////////////////////////////////
  // Setup the inertial device, ROS publishers, and subscribers
  ///////////////////////////////////////////////////////////////////////////

  try
  {
    //
    //Sometimes immediately after boot it can take a few seconds for the serial device file
    //to show up correctly in /dev.  Wait until the provided device appears before proceeeding.
    //

    ros::Rate file_check_rate(1.0);
    struct stat buffer;
    while (stat(port.c_str(), &buffer) != 0)
    {
        ROS_WARN("%s doesn't exist yet. Waiting for file to appear...", port.c_str());
        file_check_rate.sleep();
    }


    //
    //Initialize the serial interface to the device and create the inertial device object
    //

    ROS_INFO("Attempting to open serial port <%s> at <%d> \n", port.c_str(), (uint32_t)baudrate);

    mscl::Connection connection = mscl::Connection::Serial(realpath(port.c_str(), 0), (uint32_t)baudrate);
    m_inertial_device           = std::unique_ptr<mscl::InertialNode>(new mscl::InertialNode(connection));

    //Print the device info
    ROS_INFO("Model Name:    %s\n", m_inertial_device->modelName().c_str());
    ROS_INFO("Serial Number: %s\n", m_inertial_device->serialNumber().c_str());

    //Get supported features
    bool supports_gnss1  = m_inertial_device->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS) | m_inertial_device->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS1);
    bool supports_gnss2  = m_inertial_device->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS2);
    bool supports_rtk    = m_inertial_device->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS3);
    bool supports_filter = m_inertial_device->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_ESTFILTER);
    bool supports_imu    = m_inertial_device->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_AHRS_IMU);


    ///////////////////////////////////////////////////////////////////////////
    //Device Setup
    ///////////////////////////////////////////////////////////////////////////

    //If the user wants the driver to setup the device, do it now
    if(device_setup)
    {
      //Put into idle mode
      ROS_INFO("Setting to Idle: Stopping data streams and/or waking from sleep");
      m_inertial_device->setToIdle();


      //
      //GPIO config
      //

      if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_GPIO_CONFIGURATION) && gpio_config)
      {
        try {
          mscl::GpioConfiguration gpioConfig;

          gpioConfig.pin = 1;
          gpioConfig.feature = static_cast<mscl::GpioConfiguration::Feature>(gpio1_feature);
          gpioConfig.behavior = gpio1_behavior;
          gpioConfig.pinMode.value(gpio1_pin_mode);
          m_inertial_device->setGpioConfig(gpioConfig);

          ROS_INFO("Configuring GPIO1 to feature: %i, behavior: %i, pinMode: %i", gpio1_feature, gpio1_behavior, gpio1_pin_mode);

          gpioConfig.pin = 2;
          gpioConfig.feature = static_cast<mscl::GpioConfiguration::Feature>(gpio2_feature);
          gpioConfig.behavior = gpio2_behavior;
          gpioConfig.pinMode.value(gpio4_pin_mode);
          m_inertial_device->setGpioConfig(gpioConfig);

          ROS_INFO("Configuring GPIO2 to feature: %i, behavior: %i, pinMode: %i", gpio2_feature, gpio2_behavior, gpio2_pin_mode);

          gpioConfig.pin = 3;
          gpioConfig.feature = static_cast<mscl::GpioConfiguration::Feature>(gpio3_feature);
          gpioConfig.behavior = gpio3_behavior;
          gpioConfig.pinMode.value(gpio4_pin_mode);
          m_inertial_device->setGpioConfig(gpioConfig);

          ROS_INFO("Configuring GPIO3 to feature: %i, behavior: %i, pinMode: %i", gpio3_feature, gpio3_behavior, gpio3_pin_mode);

          gpioConfig.pin = 4;
          gpioConfig.feature = static_cast<mscl::GpioConfiguration::Feature>(gpio4_feature);
          gpioConfig.behavior = gpio4_behavior;
          gpioConfig.pinMode.value(gpio4_pin_mode);
          m_inertial_device->setGpioConfig(gpioConfig);

          ROS_INFO("Configuring GPIO4 to feature: %i, behavior: %i, pinMode: %i", gpio4_feature, gpio4_behavior, gpio4_pin_mode);
        }

        catch(mscl::Error &e)
        {
          ROS_ERROR("GPIO Config Error: %s", e.what());
        }
      }

      //
      //IMU Setup
      //

      if(m_publish_imu && supports_imu)
      {
        mscl::SampleRate imu_rate = mscl::SampleRate::Hertz(m_imu_data_rate);

        ROS_INFO("Setting IMU data to stream at %d hz", m_imu_data_rate);

        mscl::MipTypes::MipChannelFields ahrsChannels{
            mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_ACCEL_VEC,
            mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_GYRO_VEC,
            mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_ORIENTATION_QUATERNION,
            mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_SCALED_MAG_VEC,
            mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_GPS_CORRELATION_TIMESTAMP
           };

        mscl::MipChannels supportedChannels;
        for(mscl::MipTypes::ChannelField channel : m_inertial_device->features().supportedChannelFields(mscl::MipTypes::DataClass::CLASS_AHRS_IMU))
        {
          if(std::find(ahrsChannels.begin(), ahrsChannels.end(), channel) != ahrsChannels.end())
          {
            supportedChannels.push_back(mscl::MipChannel(channel, imu_rate));
          }
        }

        m_inertial_device->setActiveChannelFields(mscl::MipTypes::DataClass::CLASS_AHRS_IMU, supportedChannels);

        if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_DECLINATION_SRC))
        {
          ROS_INFO("Setting Declination Source");
          m_inertial_device->setDeclinationSource(mscl::GeographicSourceOptions(static_cast<mscl::InertialTypes::GeographicSourceOption>((uint8_t)declination_source), declination));
        }
        else
        {
          ROS_INFO("Note: Device does not support the declination source command.");
        }

        m_inertial_device->enableDataStream(mscl::MipTypes::DataClass::CLASS_AHRS_IMU);
      }


      //
      //GNSS1 setup
      //

      if(m_publish_gnss[GNSS1_ID] && supports_gnss1)
      {
        mscl::SampleRate gnss1_rate = mscl::SampleRate::Hertz(m_gnss_data_rate[GNSS1_ID]);

        ROS_INFO("Setting GNSS1 data to stream at %d hz", m_gnss_data_rate[GNSS1_ID]);

        mscl::MipTypes::MipChannelFields gnssChannels{
            mscl::MipTypes::ChannelField::CH_FIELD_GNSS_LLH_POSITION,
            mscl::MipTypes::ChannelField::CH_FIELD_GNSS_NED_VELOCITY,
            mscl::MipTypes::ChannelField::CH_FIELD_GNSS_GPS_TIME};

        mscl::MipTypes::DataClass gnss1_data_class = mscl::MipTypes::DataClass::CLASS_GNSS;

        if(m_inertial_device->features().supportsCategory(mscl::MipTypes::DataClass::CLASS_GNSS1))
        {
          gnss1_data_class = mscl::MipTypes::DataClass::CLASS_GNSS1;

          gnssChannels.clear();
          gnssChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_LLH_POSITION);
          gnssChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_NED_VELOCITY);
          gnssChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_GPS_TIME);
        }

        mscl::MipChannels supportedChannels;
        for (mscl::MipTypes::ChannelField channel : m_inertial_device->features().supportedChannelFields(gnss1_data_class))
        {
          if (std::find(gnssChannels.begin(), gnssChannels.end(), channel) != gnssChannels.end())
          {
            supportedChannels.push_back(mscl::MipChannel(channel, gnss1_rate));
          }
        }

        //set the GNSS channel fields
        m_inertial_device->setActiveChannelFields(gnss1_data_class, supportedChannels);

        //Set the antenna offset, if supported (needs to process 2 different ways for old devices vs. new for GNSS1)
        mscl::PositionOffset antenna_offset(m_gnss_antenna_offset[GNSS1_ID][0], m_gnss_antenna_offset[GNSS1_ID][1], m_gnss_antenna_offset[GNSS1_ID][2]);

        if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ANTENNA_OFFSET))
        {
          ROS_INFO("Setting GNSS1 antenna offset to [%f, %f, %f]", antenna_offset.x(), antenna_offset.y(), antenna_offset.z());
          m_inertial_device->setAntennaOffset(antenna_offset);
        }
        else if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_MULTI_ANTENNA_OFFSET))
        {
          ROS_INFO("Setting GNSS1 antenna offset to [%f, %f, %f]", antenna_offset.x(), antenna_offset.y(), antenna_offset.z());
          m_inertial_device->setMultiAntennaOffset(1, antenna_offset);
        }
        else
        {
          ROS_ERROR("Could not set GNSS1 antenna offset!");
        }

        m_inertial_device->enableDataStream(gnss1_data_class);
      }


      //
      //GNSS2 setup
      //

      if(m_publish_gnss[GNSS2_ID] && supports_gnss2)
      {
        mscl::SampleRate gnss2_rate = mscl::SampleRate::Hertz(m_gnss_data_rate[GNSS2_ID]);

        ROS_INFO("Setting GNSS2 data to stream at %d hz", m_gnss_data_rate[GNSS2_ID]);

        mscl::MipTypes::MipChannelFields gnssChannels{
            mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_LLH_POSITION,
            mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_NED_VELOCITY,
            mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_GPS_TIME};

        mscl::MipChannels supportedChannels;
        for (mscl::MipTypes::ChannelField channel : m_inertial_device->features().supportedChannelFields(mscl::MipTypes::DataClass::CLASS_GNSS2))
        {
          if (std::find(gnssChannels.begin(), gnssChannels.end(), channel) != gnssChannels.end())
          {
            supportedChannels.push_back(mscl::MipChannel(channel, gnss2_rate));
          }
        }

        //set the GNSS channel fields
        m_inertial_device->setActiveChannelFields(mscl::MipTypes::DataClass::CLASS_GNSS2, supportedChannels);

        //Set the antenna offset
        mscl::PositionOffset antenna_offset(m_gnss_antenna_offset[GNSS2_ID][0], m_gnss_antenna_offset[GNSS2_ID][1], m_gnss_antenna_offset[GNSS2_ID][2]);

        if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_MULTI_ANTENNA_OFFSET))
        {
          ROS_INFO("Setting GNSS2 antenna offset to [%f, %f, %f]", antenna_offset.x(), antenna_offset.y(), antenna_offset.z());
          m_inertial_device->setMultiAntennaOffset(2, antenna_offset);
        }
        else
        {
          ROS_ERROR("Could not set GNSS2 antenna offset!");
        }

        m_inertial_device->enableDataStream(mscl::MipTypes::DataClass::CLASS_GNSS2);
      }

      //
      // RTK Dongle
      //

      if(rtk_dongle_enable && supports_rtk)
      {

        mscl::SampleRate gnss3_rate = mscl::SampleRate::Hertz(1);

        ROS_INFO("Setting RTK data to stream at 1 hz");

        mscl::MipTypes::MipChannelFields gnssChannels{
            mscl::MipTypes::ChannelField::CH_FIELD_GNSS_3_RTK_CORRECTIONS_STATUS};

        mscl::MipChannels supportedChannels;
        for (mscl::MipTypes::ChannelField channel : m_inertial_device->features().supportedChannelFields(mscl::MipTypes::DataClass::CLASS_GNSS3))
        {
          if (std::find(gnssChannels.begin(), gnssChannels.end(), channel) != gnssChannels.end())
          {
            supportedChannels.push_back(mscl::MipChannel(channel, gnss3_rate));
          }
        }

        //set the GNSS channel fields
        m_inertial_device->setActiveChannelFields(mscl::MipTypes::DataClass::CLASS_GNSS3, supportedChannels);


        if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_GNSS_RTK_CONFIG))
        {
          ROS_INFO("Setting RTK dongle enable to %d", rtk_dongle_enable);
          m_inertial_device->enableRtk(rtk_dongle_enable);

          m_publish_rtk = rtk_dongle_enable;
        }
        else
        {
          ROS_INFO("Note: Device does not support the RTK dongle config command");
        }


        m_inertial_device->enableDataStream(mscl::MipTypes::DataClass::CLASS_GNSS3);
      }


      //
      //Filter setup
      //

      if(m_publish_filter && supports_filter)
      {
        mscl::SampleRate filter_rate = mscl::SampleRate::Hertz(m_filter_data_rate);

        ROS_INFO("Setting Filter data to stream at %d hz", m_filter_data_rate);

        mscl::MipTypes::MipChannelFields navChannels{
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_GPS_TIMESTAMP,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_FILTER_STATUS,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_LLH_POS,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_NED_VELOCITY,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_LLH_UNCERT,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_NED_UNCERT,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ATT_UNCERT_QUAT,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ATT_UNCERT_EULER,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_COMPENSATED_ACCEL,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_HEADING_UPDATE_SOURCE,
            mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_NED_RELATIVE_POS};

            if(filter_enable_gnss_pos_vel_aiding)
              navChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_POSITION_AIDING_STATUS);
            if(filter_enable_gnss_heading_aiding)
              navChannels.push_back(mscl::MipTypes::ChannelField::CH_FIELD_ESTFILTER_GNSS_DUAL_ANTENNA_STATUS);

        mscl::MipChannels supportedChannels;
        for(mscl::MipTypes::ChannelField channel : m_inertial_device->features().supportedChannelFields(mscl::MipTypes::DataClass::CLASS_ESTFILTER))
        {
          if(std::find(navChannels.begin(), navChannels.end(), channel) != navChannels.end())
          {
            supportedChannels.push_back(mscl::MipChannel(channel, filter_rate));
          }
        }

        m_inertial_device->setActiveChannelFields(mscl::MipTypes::DataClass::CLASS_ESTFILTER, supportedChannels);

        //set dynamics mode
        if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_VEHIC_DYNAMICS_MODE))
        {
          mscl::VehicleModeTypes modes = m_inertial_device->features().supportedVehicleModeTypes();
          if (std::find(modes.begin(), modes.end(), static_cast<mscl::InertialTypes::VehicleModeType>(dynamics_mode)) != modes.end())
          {
            ROS_INFO("Setting dynamics mode to %#04X", static_cast<mscl::InertialTypes::VehicleModeType>(dynamics_mode));
            m_inertial_device->setVehicleDynamicsMode(static_cast<mscl::InertialTypes::VehicleModeType>(dynamics_mode));
          }
        }
        else
        {
          ROS_INFO("Note: The device does not support the vehicle dynamics mode command.");
        }


        //Set PPS source
        if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_PPS_SOURCE))
        {
          mscl::PpsSourceOptions sources = m_inertial_device->features().supportedPpsSourceOptions();
          if (std::find(sources.begin(), sources.end(), static_cast<mscl::InertialTypes::PpsSource>(filter_pps_source)) != sources.end())
          {
            ROS_INFO("Setting PPS source to %#04X", static_cast<mscl::InertialTypes::PpsSource>(filter_pps_source));
            m_inertial_device->setPpsSource(static_cast<mscl::InertialTypes::PpsSource>(filter_pps_source));
          }
        }
        else
        {
          ROS_INFO("Note: The device does not support the PPS source command.");
        }

        //Set heading Source
        if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_HEADING_UPDATE_CTRL))
        {
          for(mscl::HeadingUpdateOptions headingSources : m_inertial_device->features().supportedHeadingUpdateOptions())
          {
            if(headingSources.AsOptionId() == static_cast<mscl::InertialTypes::HeadingUpdateEnableOption>(heading_source))
            {
              ROS_INFO("Setting heading source to %#04X", heading_source);
              m_inertial_device->setHeadingUpdateControl(mscl::HeadingUpdateOptions(static_cast<mscl::InertialTypes::HeadingUpdateEnableOption>(heading_source)));
              break;
            }
          }

          //Set the initial heading
          if((heading_source == 0) && (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_INIT_HEADING)))
          {
            ROS_INFO("Setting initial heading to %f", initial_heading);
            m_inertial_device->setInitialHeading(initial_heading);
          }
        }
        else
        {
          ROS_INFO("Note: The device does not support the heading source command.");
        }


        //Set the filter autoinitialization, if suppored
        if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_AUTO_INIT_CTRL))
        {
          ROS_INFO("Setting autoinitialization to %d", filter_auto_init);
          m_inertial_device->setAutoInitialization(filter_auto_init);
        }
        else
        {
          ROS_INFO("Note: The device does not support the filter autoinitialization command.");
        }


        //(GQ7 and GX5-45 only) Set the filter adaptive settings
        if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ADAPTIVE_FILTER_OPTIONS))
        {
          ROS_INFO("Setting autoadaptive options to: level = %d, time_limit = %d", filter_adaptive_level, filter_adaptive_time_limit_ms);
          mscl::AutoAdaptiveFilterOptions options(static_cast<mscl::InertialTypes::AutoAdaptiveFilteringLevel>(filter_adaptive_level), (uint16_t)filter_adaptive_time_limit_ms);

          m_inertial_device->setAdaptiveFilterOptions(options);
        }
        else
        {
          ROS_INFO("Note: The device does not support the filte adaptive settings command.");
        }


        //(GQ7 only) Set the filter aiding settings
        if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_AIDING_MEASUREMENT_ENABLE))
        {
          ROS_INFO("Filter aiding set to: pos/vel = %d, gnss heading = %d, altimeter = %d, odometer = %d, magnetometer = %d, external heading = %d",
                    filter_enable_gnss_heading_aiding, filter_enable_gnss_heading_aiding, filter_enable_altimeter_aiding, filter_enable_odometer_aiding,
                    filter_enable_magnetometer_aiding, filter_enable_external_heading_aiding);

          m_inertial_device->enableDisableAidingMeasurement(mscl::InertialTypes::AidingMeasurementSource::GNSS_POS_VEL_AIDING,     filter_enable_gnss_pos_vel_aiding);
          m_inertial_device->enableDisableAidingMeasurement(mscl::InertialTypes::AidingMeasurementSource::GNSS_HEADING_AIDING,     filter_enable_gnss_heading_aiding);
          m_inertial_device->enableDisableAidingMeasurement(mscl::InertialTypes::AidingMeasurementSource::ALTIMETER_AIDING,        filter_enable_altimeter_aiding);
          m_inertial_device->enableDisableAidingMeasurement(mscl::InertialTypes::AidingMeasurementSource::ODOMETER_AIDING,         filter_enable_odometer_aiding);
          m_inertial_device->enableDisableAidingMeasurement(mscl::InertialTypes::AidingMeasurementSource::MAGNETOMETER_AIDING,     filter_enable_magnetometer_aiding);
          m_inertial_device->enableDisableAidingMeasurement(mscl::InertialTypes::AidingMeasurementSource::EXTERNAL_HEADING_AIDING, filter_enable_external_heading_aiding);
        }
        else
        {
          ROS_INFO("Note: The device does not support the filter aiding command.");
        }


        //(GQ7 only) Set the filter relative position frame settings
        if(m_publish_filter_relative_pos && m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_RELATIVE_POSITION_REF))
        {
          mscl::PositionReferenceConfiguration ref;
          ref.position = mscl::Position(filter_relative_position_ref[0], filter_relative_position_ref[1], filter_relative_position_ref[2], static_cast<mscl::PositionVelocityReferenceFrame>(filter_relative_position_frame));

          ROS_INFO("Setting reference position to: [%f, %f, %f], ref frame = %d", filter_relative_position_ref[0], filter_relative_position_ref[1], filter_relative_position_ref[2], filter_relative_position_frame);
          m_inertial_device->setRelativePositionReference(ref);
        }
        else
        {
          ROS_INFO("Note: The device does not support the relative position command.");
        }


        //(GQ7 only) Set the filter speed lever arm
        if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SPEED_MEASUREMENT_OFFSET))
        {
          mscl::PositionOffset offset(filter_speed_lever_arm[0], filter_speed_lever_arm[1], filter_speed_lever_arm[2]);

          m_inertial_device->setSpeedMeasurementOffset(offset);
        }
        else
        {
          ROS_INFO("Note: The device does not support the filter speed lever arm command.");
        }


        //(GQ7 only) Set the wheeled vehicle constraint
        if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_WHEELED_VEHICLE_CONSTRAINT))
        {
          ROS_INFO("Setting wheeled vehicle contraint enable to %d", filter_enable_wheeled_vehicle_constraint);
          m_inertial_device->enableWheeledVehicleConstraint(filter_enable_wheeled_vehicle_constraint);
        }
        else
        {
          ROS_INFO("Note: The device does not support the wheeled vehicle constraint command.");
        }


        //(GQ7 only) Set the vertical gyro constraint
        if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_VERTICAL_GYRO_CONSTRAINT))
        {
          ROS_INFO("Setting vertical gyro contraint enable to %d", filter_enable_vertical_gyro_constraint);
          m_inertial_device->enableVerticalGyroConstraint(filter_enable_vertical_gyro_constraint);
        }
        else
        {
          ROS_INFO("Note: The device does not support the vertical gyro constraint command.");
        }


        //(GQ7 only) Set the GNSS antenna calibration settings
        if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_GNSS_ANTENNA_LEVER_ARM_CAL))
        {
          mscl::AntennaLeverArmCalConfiguration config;
          config.enabled        = filter_enable_gnss_antenna_cal;
          config.maxOffsetError = filter_gnss_antenna_cal_max_offset;

          ROS_INFO("Setting GNSS antenna calibration to: enable = %d, max_offset = %f", filter_enable_gnss_antenna_cal, filter_gnss_antenna_cal_max_offset);
          m_inertial_device->setAntennaLeverArmCal(config);
        }
        else
        {
          ROS_INFO("Note: The device does not support the GNSS antenna calibration command.");
        }


        //(GQ7 only) Set the filter initialization settings
        if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_INITIALIZATION_CONFIG))
        {
          mscl::FilterInitializationValues filter_config;

          //API Variable: autoInitialize
		      filter_config.autoInitialize = filter_auto_init;

          //API Variable: initialValuesSource
          filter_config.initialValuesSource = static_cast<mscl::FilterInitialValuesSource>(filter_init_condition_src);

          //API Variable: autoHeadingAlignmentMethod
          filter_config.autoHeadingAlignmentMethod = static_cast<mscl::HeadingAlignmentMethod>(filter_auto_heading_alignment_selector);

          //API Variable: initialAttitude
          //  Note: Only heading value will be used if initialValueSource indicates pitch/roll will be determined automatically.
          filter_config.initialAttitude = mscl::EulerAngles(filter_init_attitude[0], filter_init_attitude[1], filter_init_attitude[2]);

          //API Variable: initialPosition
          filter_config.initialPosition = mscl::Position(filter_init_position[0], filter_init_position[1], filter_init_position[2], static_cast<mscl::PositionVelocityReferenceFrame>(filter_init_reference_frame));

          //API Variable: initialVelocity
          filter_config.initialVelocity = mscl::GeometricVector(filter_init_velocity[0], filter_init_velocity[1], filter_init_velocity[2], static_cast<mscl::PositionVelocityReferenceFrame>(filter_init_reference_frame));

          //API Variable: referenceFrame
          filter_config.referenceFrame = static_cast<mscl::PositionVelocityReferenceFrame>(filter_init_reference_frame);

          m_inertial_device->setInitialFilterConfiguration(filter_config);
        }
        else
        {
          ROS_INFO("Note: The device does not support the next-gen filter initialization command.");
        }


        //Enable the filter datastream
        m_inertial_device->enableDataStream(mscl::MipTypes::DataClass::CLASS_ESTFILTER);
      }

      //
      //Set sensor2vehicle frame transformation
      //


      //Euler Angles
      if(filter_sensor2vehicle_frame_selector == 1)
      {
        //Old style - set rotation (inverse of transformation)
        if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_ROTATION_EULER))
        {
          //Invert the angles for "rotation"
          mscl::EulerAngles angles(-filter_sensor2vehicle_frame_transformation_euler[0], -filter_sensor2vehicle_frame_transformation_euler[1], -filter_sensor2vehicle_frame_transformation_euler[2]);

          ROS_INFO("Setting sensor2vehicle frame rotation with euler angles [%f, %f, %f]", -filter_sensor2vehicle_frame_transformation_euler[0], -filter_sensor2vehicle_frame_transformation_euler[1], -filter_sensor2vehicle_frame_transformation_euler[2]);
          m_inertial_device->setSensorToVehicleRotation_eulerAngles(angles);
        }
        else if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_TRANSFORM_EULER))
        {
          mscl::EulerAngles angles(filter_sensor2vehicle_frame_transformation_euler[0], filter_sensor2vehicle_frame_transformation_euler[1], filter_sensor2vehicle_frame_transformation_euler[2]);

          ROS_INFO("Setting sensor2vehicle frame transformation with euler angles [%f, %f, %f]", filter_sensor2vehicle_frame_transformation_euler[0], filter_sensor2vehicle_frame_transformation_euler[1], filter_sensor2vehicle_frame_transformation_euler[2]);
          m_inertial_device->setSensorToVehicleTransform_eulerAngles(angles);
        }
        else
        {
          ROS_ERROR("**Failed to set sensor2vehicle frame transformation with euler angles!");
        }
      }
      //Matrix
      else if(filter_sensor2vehicle_frame_selector == 2)
      {
        //Old style - set rotation (inverse of transformation)
        if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_ROTATION_DCM))
        {
          //Transpose the matrix for "rotation"
          mscl::Matrix_3x3 dcm(filter_sensor2vehicle_frame_transformation_matrix[0], filter_sensor2vehicle_frame_transformation_matrix[3], filter_sensor2vehicle_frame_transformation_matrix[6],
                               filter_sensor2vehicle_frame_transformation_matrix[1], filter_sensor2vehicle_frame_transformation_matrix[4], filter_sensor2vehicle_frame_transformation_matrix[7],
                               filter_sensor2vehicle_frame_transformation_matrix[2], filter_sensor2vehicle_frame_transformation_matrix[5], filter_sensor2vehicle_frame_transformation_matrix[8]);

          ROS_INFO("Setting sensor2vehicle frame rotation with a matrix");
          m_inertial_device->setSensorToVehicleRotation_matrix(dcm);
        }
        else if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_TRANSFORM_DCM))
        {
          mscl::Matrix_3x3 dcm(filter_sensor2vehicle_frame_transformation_matrix[0], filter_sensor2vehicle_frame_transformation_matrix[1], filter_sensor2vehicle_frame_transformation_matrix[2],
                               filter_sensor2vehicle_frame_transformation_matrix[3], filter_sensor2vehicle_frame_transformation_matrix[4], filter_sensor2vehicle_frame_transformation_matrix[5],
                               filter_sensor2vehicle_frame_transformation_matrix[6], filter_sensor2vehicle_frame_transformation_matrix[7], filter_sensor2vehicle_frame_transformation_matrix[8]);

          ROS_INFO("Setting sensor2vehicle frame transformation with a matrix");
          m_inertial_device->setSensorToVehicleTransform_matrix(dcm);
        }
        else
        {
          ROS_ERROR("**Failed to set sensor2vehicle frame transformation with a matrix!");
        }

      }
      //Quaternion
      else if(filter_sensor2vehicle_frame_selector == 3)
      {
        //Old style - set rotation (inverse of transformation)
        if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_ROTATION_QUAT))
        {
          //Invert the quaternion for "rotation" (note: device uses aerospace quaternion definition [w, -i, -j, -k])
          mscl::Quaternion quat(filter_sensor2vehicle_frame_transformation_quaternion[3], -filter_sensor2vehicle_frame_transformation_quaternion[0],
                                -filter_sensor2vehicle_frame_transformation_quaternion[1], -filter_sensor2vehicle_frame_transformation_quaternion[2]);

          ROS_INFO("Setting sensor2vehicle frame rotation with quaternion [%f %f %f %f]", -filter_sensor2vehicle_frame_transformation_quaternion[0], -filter_sensor2vehicle_frame_transformation_quaternion[1],
                                -filter_sensor2vehicle_frame_transformation_quaternion[2], filter_sensor2vehicle_frame_transformation_quaternion[3]);
          m_inertial_device->setSensorToVehicleRotation_quaternion(quat);
        }
        else if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_TRANSFORM_QUAT))
        {
          //No inversion for transformation (note: device uses aerospace quaternion definition [w, i, j, k])
          mscl::Quaternion quat(filter_sensor2vehicle_frame_transformation_quaternion[3], filter_sensor2vehicle_frame_transformation_quaternion[0],
                                filter_sensor2vehicle_frame_transformation_quaternion[1], filter_sensor2vehicle_frame_transformation_quaternion[2]);

          ROS_INFO("Setting sensor2vehicle frame transformation with quaternion [%f %f %f %f]", filter_sensor2vehicle_frame_transformation_quaternion[0], filter_sensor2vehicle_frame_transformation_quaternion[1],
                                filter_sensor2vehicle_frame_transformation_quaternion[2], filter_sensor2vehicle_frame_transformation_quaternion[3]);
          m_inertial_device->setSensorToVehicleTransform_quaternion(quat);
        }
        else
        {
          ROS_ERROR("**Failed to set sensor2vehicle frame transformation with quaternion!");
        }
      }


      //
      //Support channel setup
      //

      if(m_raw_file_enable && m_raw_file_include_support_data)
      {
        if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_FACTORY_STREAMING))
        {
          ROS_INFO("Enabling factory support channels");

          m_inertial_device->setFactoryStreamingChannels(mscl::InertialTypes::FACTORY_STREAMING_ADDITIVE);
        }
        else
        {
          ROS_ERROR("**The device does not support the factory streaming channels setup command!");
        }
      }


      //Save the settings to the device, if enabled
      if(save_settings)
      {
        ROS_INFO("Saving the launch file configuration settings to the device");
        m_inertial_device->saveSettingsAsStartup();
      }


      //Reset the filter, if enabled
      if(filter_reset_after_config && m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_RESET_FILTER))
      {
        ROS_INFO("Resetting the filter after the configuration is complete.");
        m_inertial_device->resetFilter();
      }
      else
      {
        ROS_INFO("Note: The filter was not reset after configuration.");
      }


      //Resume the device
      m_inertial_device->resume();
    }



    ///////////////////////////////////////////////////////////////////////////
    //Setup Raw Data File
    ///////////////////////////////////////////////////////////////////////////

    //Open raw data file, if enabled, and configure the device for raw data output
    if(m_raw_file_enable)
    {
      time_t raw_time;
      struct tm * curr_time;
      char curr_time_buffer[100];

      //Get the current time
      time(&raw_time);
      curr_time = localtime(&raw_time);
      strftime(curr_time_buffer, sizeof(curr_time_buffer),"%y_%m_%d_%H_%M_%S", curr_time);

      std::string time_string(curr_time_buffer);

      std::string filename = raw_file_directory + std::string("/") +
                              m_inertial_device->modelName() + std::string("_") + m_inertial_device->serialNumber() +
                              std::string("_") + time_string + std::string(".bin");

      m_raw_file.open(filename, std::ios::out | std::ios::binary | std::ios::trunc);

      if(!m_raw_file.is_open())
      {
        ROS_INFO("ERROR opening raw binary datafile at %s", filename.c_str());
      }
      else
      {
        ROS_INFO("Raw binary datafile opened at %s", filename.c_str());
      }

      m_inertial_device->connection().debugMode(true);
    }


    ///////////////////////////////////////////////////////////////////////////
    //Setup Publishers
    ///////////////////////////////////////////////////////////////////////////

    //Publishes device status, if supported
    ros::ServiceServer get_basic_status_service;
    ros::ServiceServer get_diagnostic_report_service;
    ros::ServiceServer device_report_service;

    if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
    {
      m_device_status_pub           = node.advertise<mscl_msgs::Status>("device/status", 100);
      get_basic_status_service      = node.advertiseService("get_basic_status",      &Microstrain::get_basic_status, this);
      get_diagnostic_report_service = node.advertiseService("get_diagnostic_report", &Microstrain::get_diagnostic_report, this);
    }

    if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_GET_DEVICE_INFO))
    {
      device_report_service = node.advertiseService("device_report", &Microstrain::device_report, this);
    }

    //Publish IMU data, if enabled
    if(m_publish_imu)
    {
      ROS_INFO("Publishing IMU data.");
      m_imu_pub = node.advertise<sensor_msgs::Imu>("imu/data", 100);
    }

    //Publish IMU GPS correlation data, if enabled
    if(m_publish_imu && m_publish_gps_corr)
    {
      ROS_INFO("Publishing IMU GPS correlation timestamp.");
      m_gps_corr_pub = node.advertise<mscl_msgs::GPSCorrelationTimestampStamped>("gps_corr", 100);
    }

    //If the device has a magnetometer, publish relevant topics
    if(m_publish_imu && m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_MAG_HARD_IRON_OFFSET))
    {
      ROS_INFO("Publishing Magnetometer data.");
      m_mag_pub = node.advertise<sensor_msgs::MagneticField>("mag", 100);
    }

    //If the device has GNSS1, publish relevant topics
    if(m_publish_gnss[GNSS1_ID] && supports_gnss1)
    {
      ROS_INFO("Publishing GNSS1 data.");
      m_gnss_pub[GNSS1_ID]      = node.advertise<sensor_msgs::NavSatFix>("gnss1/fix", 100);
      m_gnss_odom_pub[GNSS1_ID] = node.advertise<nav_msgs::Odometry>("gnss1/odom", 100);
      m_gnss_time_pub[GNSS1_ID] = node.advertise<sensor_msgs::TimeReference>("gnss1/time_ref", 100);

      if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_AIDING_MEASUREMENT_ENABLE))
      {
        m_gnss_aiding_status_pub[GNSS1_ID]     = node.advertise<mscl_msgs::GNSSAidingStatus>("gnss1/aiding_status", 100);
        m_publish_gnss_aiding_status[GNSS1_ID] = true;
      }
    }

    //If the device has GNSS2, publish relevant topics
    if(m_publish_gnss[GNSS2_ID] && supports_gnss2)
    {
      ROS_INFO("Publishing GNSS2 data.");
      m_gnss_pub[GNSS2_ID]      = node.advertise<sensor_msgs::NavSatFix>("gnss2/fix", 100);
      m_gnss_odom_pub[GNSS2_ID] = node.advertise<nav_msgs::Odometry>("gnss2/odom", 100);
      m_gnss_time_pub[GNSS2_ID] = node.advertise<sensor_msgs::TimeReference>("gnss2/time_ref", 100);

      if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_AIDING_MEASUREMENT_ENABLE))
      {
        m_gnss_aiding_status_pub[GNSS2_ID]     = node.advertise<mscl_msgs::GNSSAidingStatus>("gnss2/aiding_status", 100);
        m_publish_gnss_aiding_status[GNSS2_ID] = true;
      }
    }

    //If the device has RTK, publish relevant topics
    if(m_publish_rtk && supports_rtk)
    {
      ROS_INFO("Publishing RTK data.");
      m_rtk_pub =  node.advertise<mscl_msgs::RTKStatus>("rtk/status", 100);
    }

    //If the device has a kalman filter, publish relevant topics
    if(m_publish_filter && supports_filter)
    {
      ROS_INFO("Publishing Filter data.");
      m_filter_pub                   = node.advertise<nav_msgs::Odometry>("nav/odom", 100);
      m_filter_status_pub            = node.advertise<mscl_msgs::FilterStatus>("nav/status", 100);
      m_filter_heading_pub           = node.advertise<mscl_msgs::FilterHeading>("nav/heading", 100);
      m_filter_heading_state_pub     = node.advertise<mscl_msgs::FilterHeadingState>("nav/heading_state", 100);
      m_filtered_imu_pub             = node.advertise<sensor_msgs::Imu>("nav/filtered_imu/data", 100);

      if(filter_enable_gnss_heading_aiding)
      {
        m_gnss_dual_antenna_status_pub = node.advertise<mscl_msgs::GNSSDualAntennaStatus>("nav/dual_antenna_status", 100);
        m_publish_gnss_dual_antenna_status = true;
      }

      if(m_publish_filter_relative_pos)
        m_filter_relative_pos_pub = node.advertise<nav_msgs::Odometry>("nav/relative_pos/odom", 100);
    }


    ///////////////////////////////////////////////////////////////////////////
    // Setup Services
    ///////////////////////////////////////////////////////////////////////////

    //IMU tare orientation service
    ros::ServiceServer set_tare_orientation_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_TARE_ORIENT))
    {
      set_tare_orientation_service = node.advertiseService("set_tare_orientation", &Microstrain::set_tare_orientation, this);
    }

    //IMU Complementary filter service
    ros::ServiceServer set_complementary_filter_service;
    ros::ServiceServer get_complementary_filter_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_COMPLEMENTARY_FILTER_SETTINGS))
    {
      set_complementary_filter_service = node.advertiseService("set_complementary_filter", &Microstrain::set_complementary_filter, this);
      get_complementary_filter_service = node.advertiseService("get_complementary_filter", &Microstrain::get_complementary_filter, this);
    }

    //IMU sensor2vehicle frame rotation service
    ros::ServiceServer set_sensor2vehicle_rotation_service;
    ros::ServiceServer get_sensor2vehicle_rotation_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_ROTATION_EULER))
    {
      set_sensor2vehicle_rotation_service = node.advertiseService("set_sensor2vehicle_rotation", &Microstrain::set_sensor2vehicle_rotation, this);
      get_sensor2vehicle_rotation_service = node.advertiseService("get_sensor2vehicle_rotation", &Microstrain::get_sensor2vehicle_rotation, this);
    }

    //IMU sensor2vehicle frame offset service
    ros::ServiceServer set_sensor2vehicle_offset_service;
    ros::ServiceServer get_sensor2vehicle_offset_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_OFFSET))
    {
      set_sensor2vehicle_offset_service = node.advertiseService("set_sensor2vehicle_offset", &Microstrain::set_sensor2vehicle_offset, this);
      get_sensor2vehicle_offset_service = node.advertiseService("get_sensor2vehicle_offset", &Microstrain::get_sensor2vehicle_offset, this);
    }

    //IMU sensor2vehicle transformation service
    ros::ServiceServer get_sensor2vehicle_transformation_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_OFFSET) &&
        m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SENS_VEHIC_FRAME_ROTATION_EULER))
    {
      get_sensor2vehicle_transformation_service = node.advertiseService("get_sensor2vehicle_transformation", &Microstrain::get_sensor2vehicle_transformation, this);
    }

    //IMU Accel bias service
    ros::ServiceServer set_accel_bias_service;
    ros::ServiceServer get_accel_bias_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_ACCEL_BIAS))
    {
      set_accel_bias_service = node.advertiseService("set_accel_bias", &Microstrain::set_accel_bias, this);
      get_accel_bias_service = node.advertiseService("get_accel_bias", &Microstrain::get_accel_bias, this);
    }

    //IMU gyro bias service
    ros::ServiceServer set_gyro_bias_service;
    ros::ServiceServer get_gyro_bias_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_GYRO_BIAS))
    {
      set_gyro_bias_service = node.advertiseService("set_gyro_bias", &Microstrain::set_gyro_bias, this);
      get_gyro_bias_service = node.advertiseService("get_gyro_bias", &Microstrain::get_gyro_bias, this);
    }

    //IMU Gyro bias capture service
    ros::ServiceServer gyro_bias_capture_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_CAP_GYRO_BIAS))
    {
      gyro_bias_capture_service = node.advertiseService("gyro_bias_capture", &Microstrain::gyro_bias_capture, this);
    }

    //IMU Mag Hard iron offset service
    ros::ServiceServer set_hard_iron_values_service;
    ros::ServiceServer get_hard_iron_values_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_MAG_HARD_IRON_OFFSET))
    {
      set_hard_iron_values_service = node.advertiseService("set_hard_iron_values", &Microstrain::set_hard_iron_values, this);
      get_hard_iron_values_service = node.advertiseService("get_hard_iron_values", &Microstrain::get_hard_iron_values, this);
    }

    //IMU Mag Soft iron matrix service
    ros::ServiceServer get_soft_iron_matrix_service;
    ros::ServiceServer set_soft_iron_matrix_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_MAG_SOFT_IRON_MATRIX))
    {
      set_soft_iron_matrix_service = node.advertiseService("set_soft_iron_matrix", &Microstrain::set_soft_iron_matrix, this);
      get_soft_iron_matrix_service = node.advertiseService("get_soft_iron_matrix", &Microstrain::get_soft_iron_matrix, this);
    }

    //IMU Coning and sculling enable service
    ros::ServiceServer set_coning_sculling_comp_service;
    ros::ServiceServer get_coning_sculling_comp_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_CONING_SCULLING))
    {
      set_coning_sculling_comp_service = node.advertiseService("set_coning_sculling_comp", &Microstrain::set_coning_sculling_comp, this);
      get_coning_sculling_comp_service = node.advertiseService("get_coning_sculling_comp", &Microstrain::get_coning_sculling_comp, this);
    }

    //Kalman filter reset
    ros::ServiceServer reset_filter;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_RESET_FILTER))
    {
      reset_filter = node.advertiseService("reset_kf", &Microstrain::reset_filter, this);
    }

    //Kalman filter estimation control service
    ros::ServiceServer set_estimation_control_flags_service;
    ros::ServiceServer get_estimation_control_flags_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_BIAS_EST_CTRL))
    {
      set_estimation_control_flags_service = node.advertiseService("set_estimation_control_flags", &Microstrain::set_estimation_control_flags, this);
      get_estimation_control_flags_service = node.advertiseService("get_estimation_control_flags", &Microstrain::get_estimation_control_flags, this);
    }

    //Kalman filter initialization with full Euler angles service
    ros::ServiceServer init_filter_euler_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_INIT_ATTITUDE))
    {
      init_filter_euler_service = node.advertiseService("init_filter_euler", &Microstrain::init_filter_euler, this);
    }

    //Kalman filter initialization with heading only service
    ros::ServiceServer init_filter_heading_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_INIT_HEADING))
    {
      init_filter_heading_service = node.advertiseService("init_filter_heading", &Microstrain::init_filter_heading, this);
    }

    //Kalman filter heading source service
    ros::ServiceServer set_heading_source_service;
    ros::ServiceServer get_heading_source_service;
    if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_HEADING_UPDATE_CTRL))
    {
      set_heading_source_service = node.advertiseService("set_heading_source", &Microstrain::set_heading_source, this);
      get_heading_source_service = node.advertiseService("get_heading_source", &Microstrain::get_heading_source, this);
    }

    //Kalman filter commanded ZUPT service
    ros::ServiceServer commanded_vel_zupt_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_VEL_UPDATE))
    {
      commanded_vel_zupt_service = node.advertiseService("commanded_vel_zupt", &Microstrain::commanded_vel_zupt, this);
    }

    //Kalman filter commanded angular ZUPT service
    ros::ServiceServer commanded_ang_rate_zupt_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_ANG_RATE_UPDATE))
    {
      commanded_ang_rate_zupt_service = node.advertiseService("commanded_ang_rate_zupt", &Microstrain::commanded_ang_rate_zupt, this);
    }

    //Kalman filter Accel white noise 1-sigma service
    ros::ServiceServer set_accel_noise_service;
    ros::ServiceServer get_accel_noise_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ACCEL_WHT_NSE_STD_DEV))
    {
      set_accel_noise_service = node.advertiseService("set_accel_noise", &Microstrain::set_accel_noise, this);
      get_accel_noise_service = node.advertiseService("get_accel_noise", &Microstrain::get_accel_noise, this);
    }

    //Kalman filter Gyro white noise 1-sigma service
    ros::ServiceServer set_gyro_noise_service;
    ros::ServiceServer get_gyro_noise_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_GYRO_WHT_NSE_STD_DEV))
    {
      set_gyro_noise_service = node.advertiseService("set_gyro_noise", &Microstrain::set_gyro_noise, this);
      get_gyro_noise_service = node.advertiseService("get_gyro_noise", &Microstrain::get_gyro_noise, this);
    }

    //Kalman filter Mag noise 1-sigma service
    ros::ServiceServer set_mag_noise_service;
    ros::ServiceServer get_mag_noise_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_HARD_IRON_OFFSET_PROCESS_NOISE))
    {
      set_mag_noise_service = node.advertiseService("set_mag_noise", &Microstrain::set_mag_noise, this);
      get_mag_noise_service = node.advertiseService("get_mag_noise", &Microstrain::get_mag_noise, this);
    }

    //Kalman filter accel bias model service
    ros::ServiceServer set_accel_bias_model_service;
    ros::ServiceServer get_accel_bias_model_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ACCEL_BIAS_MODEL_PARAMS))
    {
      set_accel_bias_model_service = node.advertiseService("set_accel_bias_model", &Microstrain::set_accel_bias_model, this);
      get_accel_bias_model_service = node.advertiseService("get_accel_bias_model", &Microstrain::get_accel_bias_model, this);
    }

    //Kalman filter gyro bias model service
    ros::ServiceServer set_gyro_bias_model_service;
    ros::ServiceServer get_gyro_bias_model_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_GYRO_BIAS_MODEL_PARAMS))
    {
      set_gyro_bias_model_service = node.advertiseService("set_gyro_bias_model", &Microstrain::set_gyro_bias_model, this);
      get_gyro_bias_model_service = node.advertiseService("get_gyro_bias_model", &Microstrain::get_gyro_bias_model, this);
    }

    //Kalman filter magnetometer magnitude adaptive filter service
    ros::ServiceServer set_mag_adaptive_vals_service;
    ros::ServiceServer get_mag_adaptive_vals_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_MAG_MAGNITUDE_ERR_ADAPT_MEASURE))
    {
      set_mag_adaptive_vals_service = node.advertiseService("set_mag_adaptive_vals", &Microstrain::set_mag_adaptive_vals, this);
      get_mag_adaptive_vals_service = node.advertiseService("get_mag_adaptive_vals", &Microstrain::get_mag_adaptive_vals, this);
    }

    //Kalman filter magnetometer dip angle adaptive filter service
    ros::ServiceServer set_mag_dip_adaptive_vals_service;
    ros::ServiceServer get_mag_dip_adaptive_vals_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_MAG_DIP_ANGLE_ERR_ADAPT_MEASURE))
    {
      set_mag_dip_adaptive_vals_service = node.advertiseService("set_mag_dip_adaptive_vals", &Microstrain::set_mag_dip_adaptive_vals, this);
      get_mag_dip_adaptive_vals_service = node.advertiseService("get_mag_dip_adaptive_vals", &Microstrain::get_mag_dip_adaptive_vals, this);
    }

    //Kalman filter gravity adaptive filtering settings service
    ros::ServiceServer set_gravity_adaptive_vals_service;
    ros::ServiceServer get_gravity_adaptive_vals_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_GRAV_MAGNITUDE_ERR_ADAPT_MEASURE))
    {
      set_gravity_adaptive_vals_service = node.advertiseService("set_gravity_adaptive_vals", &Microstrain::set_gravity_adaptive_vals, this);
      get_gravity_adaptive_vals_service = node.advertiseService("get_gravity_adaptive_vals", &Microstrain::get_gravity_adaptive_vals, this);
    }

    //Kalman filter automatic angular ZUPT configuration service
    ros::ServiceServer set_zero_angle_update_threshold_service;
    ros::ServiceServer get_zero_angle_update_threshold_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ZERO_ANG_RATE_UPDATE_CTRL))
    {
      set_zero_angle_update_threshold_service = node.advertiseService("set_zero_angle_update_threshold", &Microstrain::set_zero_angle_update_threshold, this);
      get_zero_angle_update_threshold_service = node.advertiseService("get_zero_angle_update_threshold", &Microstrain::get_zero_angle_update_threshold, this);
    }

    //Kalman Filter automatic ZUPT configuration service
    ros::ServiceServer set_zero_velocity_update_threshold_service;
    ros::ServiceServer get_zero_velocity_update_threshold_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_ZERO_VEL_UPDATE_CTRL))
    {
      set_zero_velocity_update_threshold_service = node.advertiseService("set_zero_velocity_update_threshold", &Microstrain::set_zero_velocity_update_threshold, this);
      get_zero_velocity_update_threshold_service = node.advertiseService("get_zero_velocity_update_threshold", &Microstrain::get_zero_velocity_update_threshold, this);
    }

    //Reference position service
    ros::ServiceServer set_reference_position_service;
    ros::ServiceServer get_reference_position_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_SET_REF_POSITION))
    {
      set_reference_position_service = node.advertiseService("set_reference_position", &Microstrain::set_reference_position, this);
      get_reference_position_service = node.advertiseService("get_reference_position", &Microstrain::get_reference_position, this);
    }

    //Dynamics mode service
    ros::ServiceServer set_dynamics_mode_service;
    ros::ServiceServer get_dynamics_mode_service;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_VEHIC_DYNAMICS_MODE))
    {
      set_dynamics_mode_service = node.advertiseService("set_dynamics_mode", &Microstrain::set_dynamics_mode, this);
      get_dynamics_mode_service = node.advertiseService("get_dynamics_mode", &Microstrain::get_dynamics_mode, this);
    }


    //Device Settings Service
    ros::ServiceServer device_settings;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_SAVE_STARTUP_SETTINGS))
    {
      device_settings = node.advertiseService("device_settings", &Microstrain::device_settings, this);
    }

    //External Heading Service
    ros::ServiceServer external_heading;
    if ((m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_EXTERN_HEADING_UPDATE)) ||
        (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_EXT_HEADING_UPDATE_TS)))
    {
      external_heading = node.advertiseService("external_heading", &Microstrain::external_heading_update, this);
    }

    //Relative Position Reference Service
    ros::ServiceServer set_relative_position_reference;
    ros::ServiceServer get_relative_position_reference;
    if (m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_EXT_HEADING_UPDATE_TS))
    {
      set_relative_position_reference = node.advertiseService("set_relative_position_reference", &Microstrain::set_relative_position_reference, this);
      get_relative_position_reference = node.advertiseService("get_relative_position_reference", &Microstrain::get_relative_position_reference, this);
    }

    ///////////////////////////////////////////////////////////////////////////
    //
    //Main loop setup
    ///
    ///////////////////////////////////////////////////////////////////////////

    // Determine loop rate as 2*(max update rate), but abs. max of 1kHz
    int max_rate = std::max({m_publish_imu            ? m_imu_data_rate : 1,
                             m_publish_gnss[GNSS1_ID] ? m_gnss_data_rate[GNSS1_ID] : 1,
                             m_publish_gnss[GNSS2_ID] ? m_gnss_data_rate[GNSS2_ID] : 1,
                             m_publish_filter         ? m_filter_data_rate : 1});

    int spin_rate = std::min(2 * max_rate, 1000);

    //Set the spin rate in hz
    ROS_INFO("Setting spin rate to <%d>", spin_rate);
    ros::Rate r(spin_rate);

    ros_mscl::RosDiagnosticUpdater ros_diagnostic_updater;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    //Clear the ZUPT listener flags
    m_vel_still = false;
    m_ang_still = false;

    //Create a topic listener for ZUPTs
    if(m_velocity_zupt == 1 && m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_VEL_UPDATE))
    {
      m_filter_vel_state_sub = node.subscribe(m_velocity_zupt_topic.c_str(), 1000, &Microstrain::velocity_zupt_callback, this);
    }

    //Create a topic listener for angular ZUPTs
    if(m_angular_zupt == 1 && m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_ANG_RATE_UPDATE))
    {
      m_filter_ang_state_sub = node.subscribe(m_angular_zupt_topic.c_str(), 1000, &Microstrain::ang_zupt_callback, this);
    }

    //Create a topic listener for external GNSS updates
    if(filter_enable_external_gps_time_update && m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_GPS_TIME_UPDATE))
    {
      m_external_gps_time_sub = node.subscribe(m_external_gps_time_topic.c_str(), 1000, &Microstrain::external_gps_time_callback, this);
    }

    //
    //Main packet processing loop
    //

    ROS_INFO("Starting Data Parsing");

    uint32_t status_counter = 0;

    while(ros::ok())
    {
      mscl::MipDataPackets packets = m_inertial_device->getDataPackets(1000);

      for(mscl::MipDataPacket packet : packets)
      {
        parse_mip_packet(packet);
      }

      //Only get the status packet at 1 Hz
      if(status_counter++ >= spin_rate/2)
      {
        device_status_callback();
        status_counter = 0;
      }

      //Save raw data, if enabled
      if(m_raw_file_enable)
      {
        mscl::ConnectionDebugDataVec raw_packets = m_inertial_device->connection().getDebugData();

        for(mscl::ConnectionDebugData raw_packet : raw_packets)
        {
          const mscl::Bytes& raw_packet_bytes = raw_packet.data();

          m_raw_file.write(reinterpret_cast<const char*>(raw_packet_bytes.data()), raw_packet_bytes.size());
        }
      }

      //Take care of service requests
      ros::spinOnce();

      //Be nice
      r.sleep();
    }

  }

  catch(mscl::Error_Connection)
  {
    ROS_ERROR("Device Disconnected");
  }

  catch(mscl::Error &e)
  {
    ROS_FATAL("Error: %s", e.what());
  }

  //Release the inertial node, if necessary
  if(m_inertial_device)
  {
    m_inertial_device->setToIdle();
    m_inertial_device->connection().disconnect();
  }

  //Close raw data file if enabled
  if(m_raw_file_enable)
  {
    m_raw_file.close();
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Top-Level MIP Packet Parsing Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Microstrain::parse_mip_packet(const mscl::MipDataPacket &packet)
{
  switch (packet.descriptorSet())
  {
  case mscl::MipTypes::DataClass::CLASS_AHRS_IMU:
    parse_imu_packet(packet);
    print_packet_stats();
    break;

  case mscl::MipTypes::DataClass::CLASS_ESTFILTER:
    parse_filter_packet(packet);
    print_packet_stats();
    break;

  case mscl::MipTypes::DataClass::CLASS_GNSS:
  case mscl::MipTypes::DataClass::CLASS_GNSS1:
    parse_gnss_packet(packet, GNSS1_ID);
    print_packet_stats();
    break;

  case mscl::MipTypes::DataClass::CLASS_GNSS2:
    parse_gnss_packet(packet, GNSS2_ID);
    print_packet_stats();
    break;

  case mscl::MipTypes::DataClass::CLASS_GNSS3:
    parse_rtk_packet(packet);
    print_packet_stats();
    break;

  default:
    break;
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// MIP IMU Packet Parsing Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Microstrain::parse_imu_packet(const mscl::MipDataPacket &packet)
{
  //Update the diagnostics
  m_imu_valid_packet_count++;

  //Handle time
  uint64_t time = packet.collectedTimestamp().nanoseconds();

  //Check if the user wants to use the device timestamp instead of PC collected time
  if(packet.hasDeviceTime() && m_use_device_timestamp)
  {
     time = packet.deviceTimestamp().nanoseconds();
  }

  //IMU timestamp
  m_imu_msg.header.seq      = m_imu_valid_packet_count;
  m_imu_msg.header.stamp    = ros::Time().fromNSec(time);
  m_imu_msg.header.frame_id = m_imu_frame_id;

  //Magnetometer timestamp
  m_mag_msg.header      = m_imu_msg.header;

  //GPS correlation timestamp headder
  m_gps_corr_msg.header = m_imu_msg.header;

  //Data present flags
  bool has_accel = false;
  bool has_gyro  = false;
  bool has_quat  = false;
  bool has_mag   = false;

  //Get the list of data elements
  const mscl::MipDataPoints &points = packet.data();

  //Loop over the data elements and map them
  for(auto point_iter = points.begin(); point_iter != points.end(); point_iter++)
  {
    auto point = *point_iter;
    switch(point.field())
    {
    //Scaled Accel
    case mscl::MipTypes::CH_FIELD_SENSOR_SCALED_ACCEL_VEC:
    {
      has_accel = true;

      // Stuff into ROS message - acceleration in m/s^2
      if(point.qualifier() == mscl::MipTypes::CH_X)
      {
        m_imu_msg.linear_acceleration.x = USTRAIN_G * point.as_float();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Y)
      {
        m_imu_msg.linear_acceleration.y = USTRAIN_G * point.as_float();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Z)
      {
        m_imu_msg.linear_acceleration.z = USTRAIN_G * point.as_float();
      }
    }break;

    //Scaled Gyro
    case mscl::MipTypes::CH_FIELD_SENSOR_SCALED_GYRO_VEC:
    {
      has_gyro = true;

      if(point.qualifier() == mscl::MipTypes::CH_X)
      {
        m_imu_msg.angular_velocity.x = point.as_float();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Y)
      {
        m_imu_msg.angular_velocity.y = point.as_float();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Z)
      {
        m_imu_msg.angular_velocity.z = point.as_float();
      }
    }break;

    //Scaled Mag
    case mscl::MipTypes::CH_FIELD_SENSOR_SCALED_MAG_VEC:
    {
      has_mag = true;

      if(point.qualifier() == mscl::MipTypes::CH_X)
      {
        m_curr_imu_mag_x           = point.as_float();
        m_mag_msg.magnetic_field.x = m_curr_imu_mag_x;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Y)
      {
        m_curr_imu_mag_y           = point.as_float();
        m_mag_msg.magnetic_field.y = m_curr_imu_mag_y;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Z)
      {
        m_curr_imu_mag_z           = point.as_float();
        m_mag_msg.magnetic_field.z = m_curr_imu_mag_z;
      }
    }break;

    //Orientation Quaternion
    case mscl::MipTypes::CH_FIELD_SENSOR_ORIENTATION_QUATERNION:
    {
      has_quat = true;

      if(point.qualifier() == mscl::MipTypes::CH_QUATERNION)
      {
        mscl::Vector quaternion  = point.as_Vector();
        m_curr_filter_quaternion = quaternion;

        if(m_use_enu_frame)
        {
          tf2::Quaternion q_ned2enu, qbody2ned(quaternion.as_floatAt(1), quaternion.as_floatAt(2), quaternion.as_floatAt(3), quaternion.as_floatAt(0));
          m_t_ned2enu.getRotation(q_ned2enu);
          m_imu_msg.orientation = tf2::toMsg(q_ned2enu*qbody2ned);
        }
        else
        {

          m_imu_msg.orientation.x = quaternion.as_floatAt(1);
          m_imu_msg.orientation.y = quaternion.as_floatAt(2);
          m_imu_msg.orientation.z = quaternion.as_floatAt(3);
          m_imu_msg.orientation.w = quaternion.as_floatAt(0);
        }
      }
    }break;

    //GPS Corr
    case mscl::MipTypes::ChannelField::CH_FIELD_SENSOR_GPS_CORRELATION_TIMESTAMP:
    {
      // for some reason point.qualifier() == mscl::MipTypes::CH_WEEK_NUMBER and
      // point.qualifier() == mscl::MipTypes::CH_FLAGS always returned false so I used
      // an iterator and manually incremented it to access all the elements
      m_gps_corr_msg.gps_cor.gps_tow = point_iter->as_double();
      point_iter++;
      m_gps_corr_msg.gps_cor.gps_week_number = point_iter->as_uint16();
      point_iter++;
      m_gps_corr_msg.gps_cor.timestamp_flags = point_iter->as_uint16();
    }break;
    }
  }

  if(has_accel)
  {
    //Since the sensor does not produce a covariance for linear acceleration, set it based on our pulled in parameters.
    std::copy(m_imu_linear_cov.begin(), m_imu_linear_cov.end(), m_imu_msg.linear_acceleration_covariance.begin());
  }

  if(has_gyro)
  {
    //Since the sensor does not produce a covariance for angular velocity, set it based on our pulled in parameters.
    std::copy(m_imu_angular_cov.begin(), m_imu_angular_cov.end(), m_imu_msg.angular_velocity_covariance.begin());
  }

  if(has_quat)
  {
    //Since the MIP_AHRS data does not contain uncertainty values we have to set them based on the parameter values.
    std::copy(m_imu_orientation_cov.begin(), m_imu_orientation_cov.end(), m_imu_msg.orientation_covariance.begin());
  }

  //Publish
  if(m_publish_gps_corr)
    m_gps_corr_pub.publish(m_gps_corr_msg);

  if(m_publish_imu)
  {
    m_imu_pub.publish(m_imu_msg);

    if(has_mag)
      m_mag_pub.publish(m_mag_msg);
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// MIP Filter Packet Parsing Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Microstrain::parse_filter_packet(const mscl::MipDataPacket &packet)
{
  bool gnss_aiding_status_received[NUM_GNSS] = {false};
  bool gnss_dual_antenna_status_received = false;
  int  i;

  //Update diagnostics
  m_filter_valid_packet_count++;

  //Handle time
  uint64_t time = packet.collectedTimestamp().nanoseconds();

  //Check if the user wants to use the device timestamp instead of PC collected time
  if(packet.hasDeviceTime() && m_use_device_timestamp)
  {
     time = packet.deviceTimestamp().nanoseconds();
  }

  //Filtered IMU timestamp and frame
  m_filtered_imu_msg.header.seq      = m_filter_valid_packet_count;
  m_filtered_imu_msg.header.stamp    = ros::Time().fromNSec(time);
  m_filtered_imu_msg.header.frame_id = m_filter_frame_id;

  //Nav odom timestamp and frame
  m_filter_msg.header.seq      = m_filter_valid_packet_count;
  m_filter_msg.header.stamp    = ros::Time().fromNSec(time);
  m_filter_msg.header.frame_id = m_filter_frame_id;

  //Nav relative position odom timestamp and frame (note: Relative position frame is NED for both pos and vel)
  m_filter_relative_pos_msg.header.seq      = m_filter_valid_packet_count;
  m_filter_relative_pos_msg.header.stamp    = ros::Time().fromNSec(time);
  m_filter_relative_pos_msg.header.frame_id = m_filter_child_frame_id;
  m_filter_relative_pos_msg.child_frame_id  = m_filter_child_frame_id;

  //Get the list of data elements
  const mscl::MipDataPoints &points = packet.data();

  //Loop over data elements and map them
  for(mscl::MipDataPoint point : points)
  {
    switch(point.field())
    {
    case mscl::MipTypes::CH_FIELD_ESTFILTER_FILTER_STATUS:
    {
      if(point.qualifier() == mscl::MipTypes::CH_FILTER_STATE)
      {
        m_filter_status_msg.filter_state = point.as_uint16();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_DYNAMICS_MODE)
      {
        m_filter_status_msg.dynamics_mode = point.as_uint16();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_FLAGS)
      {
        m_filter_status_msg.status_flags = point.as_uint16();
      }
    }break;

    //Estimated LLH Position
    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_LLH_POS:
    {
      m_filter_msg.child_frame_id = m_filter_child_frame_id;

      if(point.qualifier() == mscl::MipTypes::CH_LATITUDE)
      {
        m_curr_filter_pos_lat = point.as_double();

        if(m_use_enu_frame)
        {
          m_filter_msg.pose.pose.position.y = m_curr_filter_pos_lat;
        }
        else
        {
          m_filter_msg.pose.pose.position.x = m_curr_filter_pos_lat;
        }

      }
      else if(point.qualifier() == mscl::MipTypes::CH_LONGITUDE)
      {
        m_curr_filter_pos_long = point.as_double();

        if(m_use_enu_frame)
        {
          m_filter_msg.pose.pose.position.x = m_curr_filter_pos_long;
        }
        else
        {
          m_filter_msg.pose.pose.position.y = m_curr_filter_pos_long;
        }
      }
      else if(point.qualifier() == mscl::MipTypes::CH_HEIGHT_ABOVE_ELLIPSOID)
      {
        m_curr_filter_pos_height          = point.as_double();
        m_filter_msg.pose.pose.position.z = m_curr_filter_pos_height;
      }
    }break;

    //Estimated NED Velocity
    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_NED_VELOCITY:
    {
      if(point.qualifier() == mscl::MipTypes::CH_NORTH)
      {
        m_curr_filter_vel_north = point.as_float();

        if(m_use_enu_frame)
        {
          m_filter_msg.twist.twist.linear.y              = m_curr_filter_vel_north;
          m_filter_relative_pos_msg.twist.twist.linear.y = m_curr_filter_vel_north;
        }
        else
        {
          m_filter_msg.twist.twist.linear.x              = m_curr_filter_vel_north;
          m_filter_relative_pos_msg.twist.twist.linear.x = m_curr_filter_vel_north;
        }
      }
      else if(point.qualifier() == mscl::MipTypes::CH_EAST)
      {
        m_curr_filter_vel_east = point.as_float();

        if(m_use_enu_frame)
        {
          m_filter_msg.twist.twist.linear.x              = m_curr_filter_vel_east;
          m_filter_relative_pos_msg.twist.twist.linear.x = m_curr_filter_vel_east;
        }
        else
        {
          m_filter_msg.twist.twist.linear.y              = m_curr_filter_vel_east;
          m_filter_relative_pos_msg.twist.twist.linear.y = m_curr_filter_vel_east;
        }
      }
      else if(point.qualifier() == mscl::MipTypes::CH_DOWN)
      {
        m_curr_filter_vel_down = point.as_float();

        if(m_use_enu_frame)
        {
          m_filter_msg.twist.twist.linear.z              = -m_curr_filter_vel_down;
          m_filter_relative_pos_msg.twist.twist.linear.z = -m_curr_filter_vel_down;
        }
        else
        {
          m_filter_msg.twist.twist.linear.z              = m_curr_filter_vel_down;
          m_filter_relative_pos_msg.twist.twist.linear.z = m_curr_filter_vel_down;
        }
      }
    }break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER:
    {
      if(point.qualifier() == mscl::MipTypes::CH_ROLL)
      {
        m_curr_filter_roll = point.as_float();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_PITCH)
      {
        m_curr_filter_pitch = point.as_float();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_YAW)
      {
        m_curr_filter_yaw = point.as_float();

        if(m_use_enu_frame)
        {

        }
        else
        {
          m_filter_heading_msg.heading_deg = m_curr_filter_yaw*180.0/3.14;
          m_filter_heading_msg.heading_rad = m_curr_filter_yaw;
        }
      }
      else if(point.qualifier() == mscl::MipTypes::CH_FLAGS)
      {
        m_filter_heading_msg.status_flags = point.as_uint16();
      }
    }break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_QUATERNION:
    {
      mscl::Vector quaternion  = point.as_Vector();
      m_curr_filter_quaternion = quaternion;

      if(m_use_enu_frame)
      {
        tf2::Quaternion q_ned2enu, qbody2ned(quaternion.as_floatAt(1), quaternion.as_floatAt(2), quaternion.as_floatAt(3), quaternion.as_floatAt(0));
        m_t_ned2enu.getRotation(q_ned2enu);
        m_filter_msg.pose.pose.orientation = tf2::toMsg(q_ned2enu*qbody2ned);
      }
      else
      {
        m_filter_msg.pose.pose.orientation.x  = quaternion.as_floatAt(1);
        m_filter_msg.pose.pose.orientation.y  = quaternion.as_floatAt(2);
        m_filter_msg.pose.pose.orientation.z  = quaternion.as_floatAt(3);
        m_filter_msg.pose.pose.orientation.w  = quaternion.as_floatAt(0);
      }

      m_filtered_imu_msg.orientation                  = m_filter_msg.pose.pose.orientation;
      m_filter_relative_pos_msg.pose.pose.orientation = m_filter_msg.pose.pose.orientation;
    }break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE:
    {
      if(point.qualifier() == mscl::MipTypes::CH_X)
      {
        m_curr_filter_angular_rate_x              = point.as_float();
        m_filter_msg.twist.twist.angular.x        = m_curr_filter_angular_rate_x;
        m_filtered_imu_msg.angular_velocity.x     = m_curr_filter_angular_rate_x;
        m_filter_relative_pos_msg.twist.twist.angular.x = m_curr_filter_angular_rate_x;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Y)
      {
        m_curr_filter_angular_rate_y              = point.as_float();
        m_filter_msg.twist.twist.angular.y        = m_curr_filter_angular_rate_y;
        m_filtered_imu_msg.angular_velocity.y     = m_curr_filter_angular_rate_y;
        m_filter_relative_pos_msg.twist.twist.angular.y = m_curr_filter_angular_rate_y;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Z)
      {
        m_curr_filter_angular_rate_z              = point.as_float();
        m_filter_msg.twist.twist.angular.z        = m_curr_filter_angular_rate_z;
        m_filtered_imu_msg.angular_velocity.z     = m_curr_filter_angular_rate_z;
        m_filter_relative_pos_msg.twist.twist.angular.z = m_curr_filter_angular_rate_z;
      }
    }break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_COMPENSATED_ACCEL:
    {
      if (point.qualifier() == mscl::MipTypes::CH_X)
      {
        m_filtered_imu_msg.linear_acceleration.x = point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_Y)
      {
        m_filtered_imu_msg.linear_acceleration.y = point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_Z)
      {
        m_filtered_imu_msg.linear_acceleration.z = point.as_float();
      }
    }break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_LLH_UNCERT:
    {
      if(point.qualifier() == mscl::MipTypes::CH_NORTH)
      {
        m_curr_filter_pos_uncert_north = point.as_float();

        if(m_use_enu_frame)
        {
          m_filter_msg.pose.covariance[7]              = pow(m_curr_filter_pos_uncert_north, 2);
          m_filter_relative_pos_msg.pose.covariance[7] = m_filter_msg.pose.covariance[7];
        }
        else
        {
          m_filter_msg.pose.covariance[0]              = pow(m_curr_filter_pos_uncert_north, 2);
          m_filter_relative_pos_msg.pose.covariance[0] = m_filter_msg.pose.covariance[0];
        }
      }
      else if(point.qualifier() == mscl::MipTypes::CH_EAST)
      {
        m_curr_filter_pos_uncert_east = point.as_float();

        if(m_use_enu_frame)
        {
          m_filter_msg.pose.covariance[0]              = pow(m_curr_filter_pos_uncert_east, 2);
          m_filter_relative_pos_msg.pose.covariance[0] = m_filter_msg.pose.covariance[0];
        }
        else
        {
          m_filter_msg.pose.covariance[7]              = pow(m_curr_filter_pos_uncert_east, 2);
          m_filter_relative_pos_msg.pose.covariance[7] = m_filter_msg.pose.covariance[7];
        }
      }
      else if(point.qualifier() == mscl::MipTypes::CH_DOWN)
      {
        m_curr_filter_pos_uncert_down    = point.as_float();
        m_filter_msg.pose.covariance[14] = pow(m_curr_filter_pos_uncert_down, 2);
        m_filter_relative_pos_msg.pose.covariance[14] = m_filter_msg.pose.covariance[14];
      }
    }
    break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_NED_UNCERT:
    {
      if(point.qualifier() == mscl::MipTypes::CH_NORTH)
      {
        m_curr_filter_vel_uncert_north = point.as_float();

        if(m_use_enu_frame)
        {
          m_filter_msg.twist.covariance[0]              = pow(m_curr_filter_vel_uncert_north, 2);
          m_filter_relative_pos_msg.twist.covariance[0] = m_filter_msg.twist.covariance[0];
        }
        else
        {
          m_filter_msg.twist.covariance[7]              = pow(m_curr_filter_vel_uncert_north, 2);
          m_filter_relative_pos_msg.twist.covariance[7] = m_filter_msg.twist.covariance[7];
        }

      }
      else if(point.qualifier() == mscl::MipTypes::CH_EAST)
      {
        m_curr_filter_vel_uncert_east = point.as_float();

        if(m_use_enu_frame)
        {
          m_filter_msg.twist.covariance[7]              = pow(m_curr_filter_vel_uncert_east, 2);
          m_filter_relative_pos_msg.twist.covariance[7] = m_filter_msg.twist.covariance[7];
        }
        else
        {
          m_filter_msg.twist.covariance[0]              = pow(m_curr_filter_vel_uncert_east, 2);
          m_filter_relative_pos_msg.twist.covariance[0] = m_filter_msg.twist.covariance[0];
        }

      }
      else if(point.qualifier() == mscl::MipTypes::CH_DOWN)
      {
        m_curr_filter_vel_uncert_down     = point.as_float();
        m_filter_msg.twist.covariance[14] = pow(m_curr_filter_vel_uncert_down, 2);
        m_filter_relative_pos_msg.twist.covariance[14] = m_filter_msg.twist.covariance[14];
      }
    }
    break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_ESTIMATED_ATT_UNCERT_EULER:
    {
      if(point.qualifier() == mscl::MipTypes::CH_ROLL)
      {
        m_curr_filter_att_uncert_roll                 = point.as_float();
        m_filter_msg.pose.covariance[21]              = pow(m_curr_filter_att_uncert_roll, 2);
        m_filtered_imu_msg.orientation_covariance[0]  = m_filter_msg.pose.covariance[21];
        m_filter_relative_pos_msg.pose.covariance[21] = m_filter_msg.pose.covariance[21];
      }
      else if(point.qualifier() == mscl::MipTypes::CH_PITCH)
      {
        m_curr_filter_att_uncert_pitch                = point.as_float();
        m_filter_msg.pose.covariance[28]              = pow(m_curr_filter_att_uncert_pitch, 2);
        m_filtered_imu_msg.orientation_covariance[4]  = m_filter_msg.pose.covariance[28];
        m_filter_relative_pos_msg.pose.covariance[28] = m_filter_msg.pose.covariance[28];
      }
      else if(point.qualifier() == mscl::MipTypes::CH_YAW)
      {
        m_curr_filter_att_uncert_yaw                  = point.as_float();
        m_filter_msg.pose.covariance[35]              = pow(m_curr_filter_att_uncert_yaw, 2);
        m_filtered_imu_msg.orientation_covariance[8]  = m_filter_msg.pose.covariance[35];
        m_filter_relative_pos_msg.pose.covariance[35] = m_filter_msg.pose.covariance[35];
      }
    }break;


    case mscl::MipTypes::CH_FIELD_ESTFILTER_HEADING_UPDATE_SOURCE:
    {
      if(point.qualifier() == mscl::MipTypes::CH_HEADING)
      {
        m_filter_heading_state_msg.heading_rad = point.as_float();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_HEADING_UNCERTAINTY)
      {
        m_filter_heading_state_msg.heading_uncertainty = point.as_float();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_SOURCE)
      {
        m_filter_heading_state_msg.source = point.as_uint16();
      }
      else if(point.qualifier() == mscl::MipTypes::CH_FLAGS)
      {
        m_filter_heading_state_msg.status_flags = point.as_uint16();
      }
    }break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_NED_RELATIVE_POS:
    {
      if(point.qualifier() == mscl::MipTypes::CH_X)
      {
        double rel_pos_north = point.as_double();

        if(m_use_enu_frame)
          m_filter_relative_pos_msg.pose.pose.position.y = rel_pos_north;
        else
          m_filter_relative_pos_msg.pose.pose.position.x = rel_pos_north;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Y)
      {
        double rel_pos_east = point.as_double();

        if(m_use_enu_frame)
          m_filter_relative_pos_msg.pose.pose.position.x = rel_pos_east;
        else
          m_filter_relative_pos_msg.pose.pose.position.y = rel_pos_east;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_Z)
      {
        double rel_pos_down = point.as_double();

        if(m_use_enu_frame)
          m_filter_relative_pos_msg.pose.pose.position.z = -rel_pos_down;
        else
          m_filter_relative_pos_msg.pose.pose.position.z = rel_pos_down;
      }
    }break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_POSITION_AIDING_STATUS:
    {
      if(point.hasAddlIdentifiers())
      {
        int gnss_id = static_cast<int>(point.addlIdentifiers()[0].id()) - 1;

        if(gnss_id < 0 || gnss_id >= NUM_GNSS)
        {}
        else if(point.qualifier() == mscl::MipTypes::CH_TIME_OF_WEEK)
        {
          m_gnss_aiding_status_msg[gnss_id].gps_tow = point.as_double();
          gnss_aiding_status_received[gnss_id] = true;
        }
        else if(point.qualifier() == mscl::MipTypes::CH_STATUS)
        {
          uint16_t status_flags = point.as_uint16();

          m_gnss_aiding_status_msg[gnss_id].has_position_fix         = (status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_NO_FIX) == 0;
          m_gnss_aiding_status_msg[gnss_id].tight_coupling           = status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_TIGHT_COUPLING;
          m_gnss_aiding_status_msg[gnss_id].differential_corrections = status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_DIFFERENTIAL;
          m_gnss_aiding_status_msg[gnss_id].integer_fix              = status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_INTEGER_FIX;
          m_gnss_aiding_status_msg[gnss_id].using_gps                = status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_GPS;
          m_gnss_aiding_status_msg[gnss_id].using_glonass            = status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_GLONASS;
          m_gnss_aiding_status_msg[gnss_id].using_galileo            = status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_GALILEO;
          m_gnss_aiding_status_msg[gnss_id].using_beidou             = status_flags & mscl::InertialTypes::GnssAidingStatus::GNSS_AIDING_BEIDOU;
        }
        else
        {
          ROS_INFO("Point Qualifier %d", (int)point.qualifier());
        }
      }
    }break;

    case mscl::MipTypes::CH_FIELD_ESTFILTER_GNSS_DUAL_ANTENNA_STATUS:
    {
      if (point.qualifier() == mscl::MipTypes::CH_TIME_OF_WEEK)
      {
        gnss_dual_antenna_status_received = true;
        m_gnss_dual_antenna_status_msg.gps_tow = point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_HEADING)
      {
        m_gnss_dual_antenna_status_msg.heading = point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_HEADING_UNCERTAINTY)
      {
        m_gnss_dual_antenna_status_msg.heading_uncertainty = point.as_float();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_FIX_TYPE)
      {
        m_gnss_dual_antenna_status_msg.fix_type = point.as_uint8();
      }
      else if (point.qualifier() == mscl::MipTypes::CH_STATUS)
      {
        uint16_t status_flags = point.as_uint16();

        m_gnss_dual_antenna_status_msg.rcv_1_valid           = status_flags & mscl::InertialTypes::DualAntennaStatusFlags::DATA_VALID_REC_1;
        m_gnss_dual_antenna_status_msg.rcv_2_valid           = status_flags & mscl::InertialTypes::DualAntennaStatusFlags::DATA_VALID_REC_2;
        m_gnss_dual_antenna_status_msg.antenna_offsets_valid = status_flags & mscl::InertialTypes::DualAntennaStatusFlags::ANTENNA_OFFSETS_VALID;
      }
    }break;

    default: break;
    }
  }

  //Copy fixed covariances to the filtered IMU message
  std::copy(m_imu_linear_cov.begin(),  m_imu_linear_cov.end(),  m_filtered_imu_msg.linear_acceleration_covariance.begin());
  std::copy(m_imu_angular_cov.begin(), m_imu_angular_cov.end(), m_filtered_imu_msg.angular_velocity_covariance.begin());

  //Publish
  if(m_publish_filter)
  {
    m_filtered_imu_pub.publish(m_filtered_imu_msg);
    m_filter_pub.publish(m_filter_msg);
    m_filter_status_pub.publish(m_filter_status_msg);
    m_filter_heading_pub.publish(m_filter_heading_msg);
    m_filter_heading_state_pub.publish(m_filter_heading_state_msg);

    if(m_publish_gnss_dual_antenna_status && gnss_dual_antenna_status_received)
      m_gnss_dual_antenna_status_pub.publish(m_gnss_dual_antenna_status_msg);
  }

  if(m_publish_filter_relative_pos)
    m_filter_relative_pos_pub.publish(m_filter_relative_pos_msg);

  for(i=0; i<NUM_GNSS; i++)
  {
    if(m_publish_gnss_aiding_status[i] && gnss_aiding_status_received[i])
      m_gnss_aiding_status_pub[i].publish(m_gnss_aiding_status_msg[i]);
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// MIP GNSS Packet Parsing Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Microstrain::parse_gnss_packet(const mscl::MipDataPacket &packet, int gnss_id)
{
  //Rnage-check id
  if(gnss_id >= NUM_GNSS)
    return;

  //Update diagnostics
  m_gnss_valid_packet_count[gnss_id]++;

  //Handle time
  uint64_t time       = packet.collectedTimestamp().nanoseconds();
  bool     time_valid = false;

  //Check if the user wants to use the device timestamp instead of PC collected time
  if(packet.hasDeviceTime() && m_use_device_timestamp)
  {
     time       = packet.deviceTimestamp().nanoseconds();
     time_valid = true;
  }

  //GPS Fix time
  m_gnss_msg[gnss_id].header.seq      = m_gnss_valid_packet_count[gnss_id];
  m_gnss_msg[gnss_id].header.stamp    = ros::Time().fromNSec(time);
  m_gnss_msg[gnss_id].header.frame_id = m_gnss_frame_id[gnss_id];

  //GPS Odom time
  m_gnss_odom_msg[gnss_id].header.seq      = m_gnss_valid_packet_count[gnss_id];
  m_gnss_odom_msg[gnss_id].header.stamp    = ros::Time().fromNSec(time);
  m_gnss_odom_msg[gnss_id].header.frame_id = m_gnss_frame_id[gnss_id];
  //m_gnss_odom_msg[gnss_id].child_frame_id  = m_gnss_odom_child_frame_id[gnss_id];

  //GPS Time reference
  m_gnss_time_msg[gnss_id].header.seq      = m_gnss_valid_packet_count[gnss_id];
  m_gnss_time_msg[gnss_id].header.stamp    = ros::Time::now();
  m_gnss_time_msg[gnss_id].header.frame_id = m_gnss_frame_id[gnss_id];
  m_gnss_time_msg[gnss_id].time_ref        = ros::Time().fromNSec(time);

  //Get the list of data elements
  const mscl::MipDataPoints &points = packet.data();

  //Loop over data elements and map them
  for(mscl::MipDataPoint point : points)
  {
    switch(point.field())
    {
    //LLH Position
    case mscl::MipTypes::CH_FIELD_GNSS_LLH_POSITION:
    case mscl::MipTypes::CH_FIELD_GNSS_1_LLH_POSITION:
    case mscl::MipTypes::CH_FIELD_GNSS_2_LLH_POSITION:
    {
      if(point.qualifier() == mscl::MipTypes::CH_LATITUDE)
      {
        m_gnss_msg[gnss_id].latitude = point.as_double();

        if(m_use_enu_frame)
          m_gnss_odom_msg[gnss_id].pose.pose.position.y = m_gnss_msg[gnss_id].latitude;
        else
          m_gnss_odom_msg[gnss_id].pose.pose.position.x = m_gnss_msg[gnss_id].latitude;

      }
      else if(point.qualifier() == mscl::MipTypes::CH_LONGITUDE)
      {
        m_gnss_msg[gnss_id].longitude = point.as_double();

        if(m_use_enu_frame)
          m_gnss_odom_msg[gnss_id].pose.pose.position.x = m_gnss_msg[gnss_id].longitude;
        else
          m_gnss_odom_msg[gnss_id].pose.pose.position.y = m_gnss_msg[gnss_id].longitude;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_HEIGHT_ABOVE_ELLIPSOID)
      {
        m_gnss_msg[gnss_id].altitude                  = point.as_double();
        m_gnss_odom_msg[gnss_id].pose.pose.position.z = m_gnss_msg[gnss_id].altitude;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_HORIZONTAL_ACCURACY)
      {
        //Horizontal covariance maps to lat and lon
        m_gnss_msg[gnss_id].position_covariance[0]  = pow(point.as_float(), 2);
        m_gnss_msg[gnss_id].position_covariance[4]  = m_gnss_msg[gnss_id].position_covariance[0];
        m_gnss_odom_msg[gnss_id].pose.covariance[0] = m_gnss_msg[gnss_id].position_covariance[0];
        m_gnss_odom_msg[gnss_id].pose.covariance[7] = m_gnss_msg[gnss_id].position_covariance[0];
      }
      else if(point.qualifier() == mscl::MipTypes::CH_VERTICAL_ACCURACY)
      {
        m_gnss_msg[gnss_id].position_covariance[8]   = pow(point.as_float(), 2);
        m_gnss_odom_msg[gnss_id].pose.covariance[14] = m_gnss_msg[gnss_id].position_covariance[8];
      }

      m_gnss_msg[gnss_id].status.service           = 1;
      m_gnss_msg[gnss_id].position_covariance_type = 2;
    }break;

    case mscl::MipTypes::ChannelField::CH_FIELD_GNSS_NED_VELOCITY:
    case mscl::MipTypes::ChannelField::CH_FIELD_GNSS_1_NED_VELOCITY:
    case mscl::MipTypes::ChannelField::CH_FIELD_GNSS_2_NED_VELOCITY:
    {
      if(point.qualifier() == mscl::MipTypes::CH_NORTH)
      {
        float north_velocity = point.as_float();

        if(m_use_enu_frame)
          m_gnss_odom_msg[gnss_id].twist.twist.linear.y = north_velocity;
        else
          m_gnss_odom_msg[gnss_id].twist.twist.linear.x = north_velocity;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_EAST)
      {
        float east_velocity = point.as_float();

        if(m_use_enu_frame)
          m_gnss_odom_msg[gnss_id].twist.twist.linear.x = east_velocity;
        else
          m_gnss_odom_msg[gnss_id].twist.twist.linear.y = east_velocity;
      }
      else if(point.qualifier() == mscl::MipTypes::CH_DOWN)
      {
        float down_velocity = point.as_float();

        if(m_use_enu_frame)
          m_gnss_odom_msg[gnss_id].twist.twist.linear.z = -down_velocity;
        else
          m_gnss_odom_msg[gnss_id].twist.twist.linear.z = down_velocity;
      }
    }break;
    }
  }

  //Publish
  if(m_publish_gnss[gnss_id])
  {
    m_gnss_pub[gnss_id].publish(m_gnss_msg[gnss_id]);
    m_gnss_odom_pub[gnss_id].publish(m_gnss_odom_msg[gnss_id]);

    if(time_valid)
      m_gnss_time_pub[gnss_id].publish(m_gnss_time_msg[gnss_id]);
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// MIP RTK Packet Parsing Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Microstrain::parse_rtk_packet(const mscl::MipDataPacket& packet)
{
  //Update diagnostics
  m_rtk_valid_packet_count++;

  //Handle time
  uint64_t time = packet.collectedTimestamp().nanoseconds();

  //Check if the user wants to use the device timestamp instead of PC collected time
  if(packet.hasDeviceTime() && m_use_device_timestamp)
    time = packet.deviceTimestamp().nanoseconds();

  //Get the list of data elements
  const mscl::MipDataPoints &points = packet.data();

  //Loop over data elements and map them
  for(mscl::MipDataPoint point : points)
  {
   switch(point.field())
    {
      //RTK Correction Status
      case mscl::MipTypes::CH_FIELD_GNSS_3_RTK_CORRECTIONS_STATUS:
      {
        if(point.qualifier() == mscl::MipTypes::CH_TIME_OF_WEEK)
        {
          m_rtk_msg.gps_tow = point.as_double();
        }
        else if(point.qualifier() == mscl::MipTypes::CH_WEEK_NUMBER)
        {
          m_rtk_msg.gps_week = point.as_uint16();
        }
        else if(point.qualifier() == mscl::MipTypes::CH_STATUS)
        {
          m_rtk_msg.epoch_status = point.as_uint16();
        }
        else if(point.qualifier() == mscl::MipTypes::CH_FLAGS)
        {
          //Decode dongle status
          mscl::RTKDeviceStatusFlags dongle_status(point.as_uint32());

          m_rtk_msg.dongle_controller_state  = dongle_status.controllerState();
          m_rtk_msg.dongle_platform_state 	 = dongle_status.platformState();
          m_rtk_msg.dongle_controller_status = dongle_status.controllerStatusCode();
          m_rtk_msg.dongle_platform_status 	 = dongle_status.platformStatusCode();
          m_rtk_msg.dongle_reset_reason 	   = dongle_status.resetReason();
          m_rtk_msg.dongle_signal_quality		 = dongle_status.signalQuality();
        }
        else if(point.qualifier() == mscl::MipTypes::CH_GPS_CORRECTION_LATENCY)
        {
          m_rtk_msg.gps_correction_latency = point.as_float();
        }
        else if(point.qualifier() == mscl::MipTypes::CH_GLONASS_CORRECTION_LATENCY)
        {
          m_rtk_msg.glonass_correction_latency = point.as_float();
        }
        else if(point.qualifier() == mscl::MipTypes::CH_GALILEO_CORRECTION_LATENCY)
        {
          m_rtk_msg.galileo_correction_latency = point.as_float();
        }
        else if(point.qualifier() == mscl::MipTypes::CH_BEIDOU_CORRECTION_LATENCY)
        {
          m_rtk_msg.beidou_correction_latency = point.as_float();
        }
      }break;
    }
  }

  //Publish
  if(m_publish_rtk)
    m_rtk_pub.publish(m_rtk_msg);
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Device Report
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::device_report(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Model Name       => %s\n",   m_inertial_device->modelName().c_str());
      ROS_INFO("Model Number     => %s\n",   m_inertial_device->modelNumber().c_str());
      ROS_INFO("Serial Number    => %s\n",   m_inertial_device->serialNumber().c_str());
      ROS_INFO("Options          => %s\n",   m_inertial_device->deviceOptions().c_str());
      ROS_INFO("Firmware Version => %s\n\n", m_inertial_device->firmwareVersion().str().c_str());
      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Velocity ZUPT Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Microstrain::velocity_zupt_callback(const std_msgs::Bool& state)
{
  if(m_vel_still != state.data)
  {
    m_vel_still = state.data;

    if(m_vel_still)
    {
      vel_zupt();
      m_vel_still = false;
    }
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Velocity ZUPT Subtask
//
// Note: Handles sending the ZUPT command regularly while stationary
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Microstrain::vel_zupt()
{
  ros::Rate loop_rate(5);
  ros::AsyncSpinner spinner(4);

  while(ros::ok() && m_vel_still)
  {
    if(m_inertial_device && m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_VEL_UPDATE))
    {
      try
      {
        m_inertial_device->cmdedVelZUPT();
      }
      catch (mscl::Error &e)
      {
        ROS_ERROR("Error: %s", e.what());
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Angular Rate ZUPT Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Microstrain::ang_zupt_callback(const std_msgs::Bool& state)
{
  if(m_ang_still != state.data)
  {
    m_ang_still = state.data;

    if(m_ang_still)
    {
      ang_zupt();
      m_ang_still = false;
    }
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Angular Rate ZUPT Subtask
//
// Note: Handles sending the ZUPT command regularly while not rotating
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Microstrain::ang_zupt()
{
  ros::Rate loop_rate(5);
  ros::AsyncSpinner spinner(4);

  while(ros::ok() && m_ang_still)
  {
    if(m_inertial_device && m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_VEL_UPDATE))
    {
      try
      {
        m_inertial_device->cmdedAngRateZUPT();
      }
      catch (mscl::Error &e)
      {
        ROS_ERROR("Error: %s", e.what());
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// External GPS Time Callback
/////////////////////////////////////////////////////////////////////////////////////////////////////
void Microstrain::external_gps_time_callback(const sensor_msgs::TimeReference& time)
{
  if(m_inertial_device)
  {
    try
    {
      long utcTime = time.time_ref.toSec() + m_gps_leap_seconds - UTC_GPS_EPOCH_DUR;

      long secs = utcTime % (int)SECS_PER_WEEK;

      int weeks = (utcTime - secs)/SECS_PER_WEEK;


      m_inertial_device->setGPSTimeUpdate(mscl::MipTypes::TimeFrame::TIME_FRAME_WEEKS, weeks);
      m_inertial_device->setGPSTimeUpdate(mscl::MipTypes::TimeFrame::TIME_FRAME_SECONDS, secs);

      ROS_INFO("GPS Update: w%i, s%ld", weeks, secs);
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Reset Filter Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::reset_filter(std_srvs::Empty::Request &req,
                               std_srvs::Empty::Response &res)
{
  ROS_INFO("Resetting filter\n");

  if(m_inertial_device)
  {
    try
    {
      m_inertial_device->resetFilter();
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialize Filter (Euler Angles) Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::init_filter_euler(ros_mscl::InitFilterEuler::Request &req,
                                    ros_mscl::InitFilterEuler::Response &res)
{
  res.success = false;
  ROS_INFO("Initializing the Filter with Euler angles\n");

  if(m_inertial_device)
  {
    try
    {
      mscl::EulerAngles attitude(req.angle.x,
                                 req.angle.y,
                                 req.angle.z);

      m_inertial_device->setInitialAttitude(attitude);
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialize Filter (Heading Angle) Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::init_filter_heading(ros_mscl::InitFilterHeading::Request &req,
                                      ros_mscl::InitFilterHeading::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Initializing the Filter with a heading angle\n");
      m_inertial_device->setInitialHeading(req.angle);

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Accel Bias Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_accel_bias(ros_mscl::SetAccelBias::Request &req,
                                 ros_mscl::SetAccelBias::Response &res)
{
  res.success = false;
  ROS_INFO("Setting accel bias values");

  if(m_inertial_device)
  {
    try
    {
      mscl::GeometricVector biasVector = m_inertial_device->getAccelerometerBias();

      ROS_INFO("Accel bias vector values are: %f %f %f",
               biasVector.x(), biasVector.y(), biasVector.z());
      ROS_INFO("Client request values are: %.2f %.2f %.2f",
               req.bias.x, req.bias.y, req.bias.z);

      biasVector.x(req.bias.x);
      biasVector.y(req.bias.y);
      biasVector.z(req.bias.z);

      m_inertial_device->setAccelerometerBias(biasVector);

      ROS_INFO("New accel bias vector values are: %.2f %.2f %.2f",
               biasVector.x(), biasVector.y(), biasVector.z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Accel Bias Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_accel_bias(ros_mscl::GetAccelBias::Request &req,
                                 ros_mscl::GetAccelBias::Response &res)
{
  res.success = false;
  ROS_INFO("Getting accel bias values\n");

  if(m_inertial_device)
  {
    try
    {
      mscl::GeometricVector biasVector = m_inertial_device->getAccelerometerBias();

      ROS_INFO("Accel bias vector values are: %f %f %f.\n",
               biasVector.x(), biasVector.y(), biasVector.z());

      res.bias.x = biasVector.x();
      res.bias.y = biasVector.y();
      res.bias.z = biasVector.z();

     res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }
  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Gyro Bias Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_gyro_bias(ros_mscl::SetGyroBias::Request &req,
                                ros_mscl::SetGyroBias::Response &res)
{
  res.success = false;
  ROS_INFO("Setting gyro bias values");

  if(m_inertial_device)
  {
    try
    {
      mscl::GeometricVector biasVector = m_inertial_device->getGyroBias();

      ROS_INFO("Gyro bias vector values are: %f %f %f",
               biasVector.x(), biasVector.y(), biasVector.z());

      ROS_INFO("Client request values are: %.2f %.2f %.2f",
               req.bias.x, req.bias.y, req.bias.z);

      biasVector.x(req.bias.x);
      biasVector.y(req.bias.y);
      biasVector.z(req.bias.z);

      m_inertial_device->setGyroBias(biasVector);

      ROS_INFO("New gyro bias vector values are: %.2f %.2f %.2f",
               biasVector.x(), biasVector.y(), biasVector.z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Gyro Bias Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_gyro_bias(ros_mscl::GetGyroBias::Request &req,
                                ros_mscl::GetGyroBias::Response &res)
{
  res.success = false;
  ROS_INFO("Getting gyro bias values");
  if(m_inertial_device)
  {
    try
    {
      mscl::GeometricVector biasVector = m_inertial_device->getGyroBias();

      ROS_INFO("Gyro bias vector values are: %f %f %f",
               biasVector.x(), biasVector.y(), biasVector.z());

      res.bias.x = biasVector.x();
      res.bias.y = biasVector.y();
      res.bias.z = biasVector.z();

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Gyro Bias Capture Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::gyro_bias_capture(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = false;
  ROS_INFO("Performing Gyro Bias capture.\nPlease keep device stationary during the 10 second gyro bias capture interval\n");

  if(m_inertial_device)
  {
    try
    {
      mscl::GeometricVector biasVector = m_inertial_device->captureGyroBias(10000);

      ROS_INFO("Gyro Bias Captured:\nbias_vector[0] = %f\nbias_vector[1] = %f\nbias_vector[2] = %f\n\n",
               biasVector.x(), biasVector.y(), biasVector.z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Hard Iron Offset Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_hard_iron_values(ros_mscl::SetHardIronValues::Request &req,
                                       ros_mscl::SetHardIronValues::Response &res)
{
  res.success = false;
  ROS_INFO("Setting hard iron values");

  if(m_inertial_device)
  {
    try
    {
      mscl::GeometricVector biasVector = m_inertial_device->getMagnetometerHardIronOffset();

      ROS_INFO("Hard Iron vector values are: %f %f %f",
               biasVector.x(), biasVector.y(), biasVector.z());
      ROS_INFO("Client request values are: %.2f %.2f %.2f",
               req.bias.x, req.bias.y, req.bias.z);

      biasVector.x(req.bias.x);
      biasVector.y(req.bias.y);
      biasVector.z(req.bias.z);

      m_inertial_device->setMagnetometerHardIronOffset(biasVector);

      ROS_INFO("New hard iron values are: %.2f %.2f %.2f",
               biasVector.x(), biasVector.y(), biasVector.z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Hard Iron Offset Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_hard_iron_values(ros_mscl::GetHardIronValues::Request &req,
                                       ros_mscl::GetHardIronValues::Response &res)
{
  res.success = false;
  ROS_INFO("Getting gyro bias values");

  if(m_inertial_device)
  {
    try
    {
      mscl::GeometricVector biasVector = m_inertial_device->getMagnetometerHardIronOffset();

      ROS_INFO("Hard iron values are: %f %f %f",
               biasVector.x(), biasVector.y(), biasVector.z());

      res.bias.x = biasVector.x();
      res.bias.y = biasVector.y();
      res.bias.z = biasVector.z();

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Soft Iron Matrix Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_soft_iron_matrix(ros_mscl::SetSoftIronMatrix::Request &req,
                                       ros_mscl::SetSoftIronMatrix::Response &res)
{
  res.success = false;
  ROS_INFO("Setting the soft iron matrix values\n");

  if(m_inertial_device)
  {
    try
    {
      mscl::Matrix_3x3 data;
      data.set(0, 0, req.soft_iron_1.x);
      data.set(0, 1, req.soft_iron_1.y);
      data.set(0, 2, req.soft_iron_1.z);
      data.set(1, 0, req.soft_iron_2.x);
      data.set(1, 1, req.soft_iron_2.y);
      data.set(1, 2, req.soft_iron_2.z);
      data.set(2, 0, req.soft_iron_3.x);
      data.set(2, 1, req.soft_iron_3.y);
      data.set(2, 2, req.soft_iron_3.z);

      m_inertial_device->setMagnetometerSoftIronMatrix(data);
      ROS_INFO("Sent values:     [%f  %f  %f][%f  %f  %f][%f  %f  %f]\n",
               data(0, 0), data(0, 1), data(0, 2),
               data(1, 0), data(1, 1), data(1, 2),
               data(2, 0), data(2, 1), data(2, 2));

      data = m_inertial_device->getMagnetometerSoftIronMatrix();

      ROS_INFO("Returned values:     [%f  %f  %f][%f  %f  %f][%f  %f  %f]\n",
               data(0, 0), data(0, 1), data(0, 2),
               data(1, 0), data(1, 1), data(1, 2),
               data(2, 0), data(2, 1), data(2, 2));

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Soft Iron Matrix Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_soft_iron_matrix(ros_mscl::GetSoftIronMatrix::Request &req,
                                       ros_mscl::GetSoftIronMatrix::Response &res)
{
  res.success = false;
  ROS_INFO("Getting the soft iron matrix values\n");

  if(m_inertial_device)
  {
    try
    {
      mscl::Matrix_3x3 data = m_inertial_device->getMagnetometerSoftIronMatrix();

      ROS_INFO("Soft iron matrix values: [%f  %f  %f][%f  %f  %f][%f  %f  %f]\n",
               data(0, 0), data(0, 1), data(0, 2),
               data(1, 0), data(1, 1), data(1, 2),
               data(2, 0), data(2, 1), data(2, 2));

      res.soft_iron_1.x = data(0, 0);
      res.soft_iron_1.y = data(0, 1);
      res.soft_iron_1.z = data(0, 2);
      res.soft_iron_2.x = data(1, 0);
      res.soft_iron_2.y = data(1, 1);
      res.soft_iron_2.z = data(1, 2);
      res.soft_iron_3.x = data(2, 0);
      res.soft_iron_3.y = data(2, 1);
      res.soft_iron_3.z = data(2, 2);

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Complementary Filter Settings Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_complementary_filter(ros_mscl::SetComplementaryFilter::Request &req,
                                           ros_mscl::SetComplementaryFilter::Response &res)
{
  ROS_INFO("Setting the complementary filter values\n");
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      mscl::ComplementaryFilterData comp_filter_command;
      comp_filter_command.upCompensationEnabled          = req.up_comp_enable;
      comp_filter_command.upCompensationTimeInSeconds    = req.up_comp_time_const;
      comp_filter_command.northCompensationEnabled       = req.north_comp_enable;
      comp_filter_command.northCompensationTimeInSeconds = req.north_comp_time_const;

      m_inertial_device->setComplementaryFilterSettings(comp_filter_command);

      ROS_INFO("Sent values:     Up Enable: %d North Enable: %d Up Time Constant: %f North Time Constant: %f \n",
               comp_filter_command.upCompensationEnabled, comp_filter_command.northCompensationEnabled,
               comp_filter_command.upCompensationTimeInSeconds, comp_filter_command.northCompensationTimeInSeconds);

      comp_filter_command = m_inertial_device->getComplementaryFilterSettings();

      ROS_INFO("Returned values: Up Enable: %d North Enable: %d Up Time Constant: %f North Time Constant: %f \n",
               comp_filter_command.upCompensationEnabled, comp_filter_command.northCompensationEnabled,
               comp_filter_command.upCompensationTimeInSeconds, comp_filter_command.northCompensationTimeInSeconds);

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Complementary Filter Settings Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_complementary_filter(ros_mscl::GetComplementaryFilter::Request &req,
                                           ros_mscl::GetComplementaryFilter::Response &res)
{
  res.success = false;
  ROS_INFO("Getting the complementary filter values\n");

  if(m_inertial_device)
  {
    try
    {
      mscl::ComplementaryFilterData comp_filter_command = m_inertial_device->getComplementaryFilterSettings();

      ROS_INFO("Returned values: Up Enable: %d North Enable: %d Up Time Constant: %f North Time Constant: %f \n",
               comp_filter_command.upCompensationEnabled, comp_filter_command.northCompensationEnabled,
               comp_filter_command.upCompensationTimeInSeconds, comp_filter_command.northCompensationTimeInSeconds);

      res.up_comp_enable        = comp_filter_command.upCompensationEnabled;
      res.up_comp_time_const    = comp_filter_command.upCompensationTimeInSeconds;
      res.north_comp_enable     = comp_filter_command.northCompensationEnabled;
      res.north_comp_time_const = comp_filter_command.northCompensationTimeInSeconds ;

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Heading Source Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_heading_source(ros_mscl::SetHeadingSource::Request &req,
                                     ros_mscl::SetHeadingSource::Response &res)
{
  res.success = false;
  ROS_INFO("Set Heading Source\n");

  if(m_inertial_device)
  {
    try
    {
      mscl::InertialTypes::HeadingUpdateEnableOption source = static_cast<mscl::InertialTypes::HeadingUpdateEnableOption>(req.headingSource);

      for(mscl::HeadingUpdateOptions headingSources : m_inertial_device->features().supportedHeadingUpdateOptions())
      {
        if(headingSources.AsOptionId() == source)
        {
          ROS_INFO("Setting heading source to %#04X", source);
          m_inertial_device->setHeadingUpdateControl(mscl::HeadingUpdateOptions(source));
          res.success = true;
          break;
        }
      }
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Heading Source Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_heading_source(ros_mscl::GetHeadingSource::Request &req,
                                     ros_mscl::GetHeadingSource::Response &res)
{
  res.success = false;
  ROS_INFO("Getting the heading source\n");

  if(m_inertial_device)
  {
    try
    {
      mscl::HeadingUpdateOptions source = m_inertial_device->getHeadingUpdateControl();

      ROS_INFO("Current heading source is %#04X", source.AsOptionId());

      res.headingSource = static_cast<uint8_t>(source.AsOptionId());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Sensor2Vehicle Frame Rotation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_sensor2vehicle_rotation(ros_mscl::SetSensor2VehicleRotation::Request &req,
                                              ros_mscl::SetSensor2VehicleRotation::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Setting the sensor to vehicle frame rotation\n");
      mscl::EulerAngles angles(req.angle.x, req.angle.y, req.angle.z);
      m_inertial_device->setSensorToVehicleRotation_eulerAngles(angles);

      angles = m_inertial_device->getSensorToVehicleRotation_eulerAngles();

      ROS_INFO("Rotation successfully set.\n");
      ROS_INFO("New angles: %f roll %f pitch %f yaw\n",
               angles.roll(), angles.pitch(), angles.yaw());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Sensor2Vehicle Frame Rotation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_sensor2vehicle_rotation(ros_mscl::GetSensor2VehicleRotation::Request &req,
                                              ros_mscl::GetSensor2VehicleRotation::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      mscl::EulerAngles angles = m_inertial_device->getSensorToVehicleRotation_eulerAngles();
      ROS_INFO("Sensor Vehicle Frame Rotation Angles: %f roll %f pitch %f yaw\n",
               angles.roll(), angles.pitch(), angles.yaw());

      res.angle.x = angles.roll();
      res.angle.y = angles.pitch();
      res.angle.z = angles.yaw();

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Sensor2Vehicle Frame Offset Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

// Set sensor to vehicle frame offset. Only in 45
bool Microstrain::set_sensor2vehicle_offset(ros_mscl::SetSensor2VehicleOffset::Request &req,
                                            ros_mscl::SetSensor2VehicleOffset::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Setting the sensor to vehicle frame offset\n");
      mscl::PositionOffset offset(req.offset.x, req.offset.y, req.offset.z);
      m_inertial_device->setSensorToVehicleOffset(offset);

      offset = m_inertial_device->getSensorToVehicleOffset();
      ROS_INFO("Offset successfully set.\n");
      ROS_INFO("Returned offset: %f X %f Y %f Z\n",
               offset.x(), offset.y(), offset.z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Sensor2Vehicle Frame Offset Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_sensor2vehicle_offset(ros_mscl::GetSensor2VehicleOffset::Request &req,
                                            ros_mscl::GetSensor2VehicleOffset::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Getting the sensor to vehicle frame offset\n");

      mscl::PositionOffset offset = m_inertial_device->getSensorToVehicleOffset();
      ROS_INFO("Returned offset: %f X %f Y %f Z\n",
               offset.x(), offset.y(), offset.z());

      res.offset.x = offset.x();
      res.offset.y = offset.y();
      res.offset.z = offset.z();

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Sensor2Vehicle Frame Transformation (Combination of Offset and Rotation) Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_sensor2vehicle_transformation(ros_mscl::GetSensor2VehicleTransformation::Request &req,
                                                    ros_mscl::GetSensor2VehicleTransformation::Response &res)
{
  res.success = false;

  if(!m_inertial_device)
  {
    return res.success;
  }

  try
  {
    ROS_INFO("Getting transform from sensor frame to vehicle frame");
    const mscl::PositionOffset offset = m_inertial_device->getSensorToVehicleOffset();
    const mscl::EulerAngles rotation  = m_inertial_device->getSensorToVehicleRotation_eulerAngles();

    //set offset components from the device-stored values
    res.offset.x = offset.x();
    res.offset.y = offset.y();
    res.offset.z = offset.z();

    //set rotational components from the device-stored values
    tf2::Quaternion quat;
    quat.setRPY(rotation.roll(), rotation.pitch(), rotation.yaw());
    tf2::convert(quat, res.rotation);

    res.success = true;
  }
  catch(mscl::Error &e)
  {
    ROS_ERROR("Error getting sensor to vehicle transform: '%s'", e.what());
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Reference Position Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_reference_position(ros_mscl::SetReferencePosition::Request &req,
                                         ros_mscl::SetReferencePosition::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Setting reference Position\n");

      mscl::Position referencePosition(req.position.x, req.position.y, req.position.z);
      mscl::FixedReferencePositionData referencePositionData(true, referencePosition);

      m_inertial_device->setFixedReferencePosition(referencePositionData);

      ROS_INFO("Reference position successfully set\n");
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Reference Position Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_reference_position(ros_mscl::GetReferencePosition::Request &req,
                                         ros_mscl::GetReferencePosition::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Getting reference position");

      mscl::Position referencePosition = m_inertial_device->getFixedReferencePosition().referencePosition;
      ROS_INFO("Reference position: Lat %f , Long %f, Alt %f", referencePosition.latitude(),
               referencePosition.longitude(), referencePosition.altitude());

      res.position.x = referencePosition.latitude();
      res.position.y = referencePosition.longitude();
      res.position.z = referencePosition.altitude();

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Enable/Disable Coning and Sculling Compensation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_coning_sculling_comp(ros_mscl::SetConingScullingComp::Request &req,
                                           ros_mscl::SetConingScullingComp::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("%s Coning and Sculling compensation", req.enable ? "DISABLED" : "ENABLED\n");
      m_inertial_device->setConingAndScullingEnable(req.enable);

      ROS_INFO("Reading Coning and Sculling compensation enabled state:\n");

      bool enabled = m_inertial_device->getConingAndScullingEnable();
      ROS_INFO("%s Coning and Sculling compensation", enabled ? "DISABLED" : "ENABLED\n");

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Enable/Disable Coning and Sculling Compensation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_coning_sculling_comp(ros_mscl::GetConingScullingComp::Request &req,
                                           ros_mscl::GetConingScullingComp::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Reading Coning and Sculling compensation enabled state:\n");

      bool enabled = m_inertial_device->getConingAndScullingEnable();
      ROS_INFO("%s Coning and Sculling compensation", enabled ? "DISABLED" : "ENABLED\n");

      res.enable  = enabled;
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Estimation Control Flags Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_estimation_control_flags(ros_mscl::SetEstimationControlFlags::Request &req,
                                               ros_mscl::SetEstimationControlFlags::Response &res)
{
  if(m_inertial_device)
  {
    try
    {
      mscl::EstimationControlOptions flags(req.flags);
      m_inertial_device->setEstimationControlFlags(flags);
      flags = m_inertial_device->getEstimationControlFlags();
      ROS_INFO("Estimation control set to: %d", flags.AsUint16());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Estimation Control Flags Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_estimation_control_flags(ros_mscl::GetEstimationControlFlags::Request &req,
                                               ros_mscl::GetEstimationControlFlags::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      uint16_t flags = m_inertial_device->getEstimationControlFlags().AsUint16();

      ROS_INFO("Estimation control set to: %x", flags);

      res.flags   = flags;
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Device Basic Status Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_basic_status(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  if(!m_inertial_device)
  {
    return false;
  }

  if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    mscl::DeviceStatusMap status;

    if(m_inertial_device->features().supportedStatusSelectors().size() > 0)
    {
      mscl::DeviceStatusData statusData = m_inertial_device->getBasicDeviceStatus();
      status = statusData.asMap();
    }
    else
    {
      ROS_INFO("Model Number: \t\t\t\t\t%s\n", m_inertial_device->modelNumber().c_str());
      return true;
    }

    mscl::DeviceStatusMap::iterator it;

    for(it = status.begin(); it != status.end(); it++ )
    {
      switch (it->first)
      {
      case mscl::DeviceStatusValues::ModelNumber:
        ROS_INFO("Model Number: \t\t\t\t\t%s\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::StatusStructure_Value:
        ROS_INFO("Status Selector: \t\t\t\t%s\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::SystemState_Value:
        ROS_INFO("System state: \t\t\t\t\t%s\n", (it->second).c_str());
        break;

      default:
        break;
      }
    }
  }
  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Diagnostic Status Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_diagnostic_report(std_srvs::Trigger::Request &req,
                                        std_srvs::Trigger::Response &res)
{
  res.success = false;

  if(!m_inertial_device)
  {
    return false;
  }

  if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    mscl::DeviceStatusMap status;

    if(m_inertial_device->features().supportedStatusSelectors().size() > 1)
    {
      mscl::DeviceStatusData statusData = m_inertial_device->getDiagnosticDeviceStatus();
      status = statusData.asMap();
      res.success = true;
    }

    else if(m_inertial_device->features().supportedStatusSelectors().size() > 0)
    {
      mscl::DeviceStatusData statusData = m_inertial_device->getBasicDeviceStatus();
      status = statusData.asMap();
      res.success = true;
    }
    else
    {
      ROS_INFO("Model Number: \t\t\t\t\t%s\n", m_inertial_device->modelNumber().c_str());
      return true;
    }

    mscl::DeviceStatusMap::iterator it;

    for( it = status.begin(); it != status.end(); it++ )
    {
      switch(it->first)
      {
      case mscl::DeviceStatusValues::ModelNumber:
        ROS_INFO("Model Number: \t\t\t\t\t%s\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::StatusStructure_Value:
        ROS_INFO("Status Selector: \t\t\t\t%s\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::SystemState_Value:
        ROS_INFO("System state: \t\t\t\t\t%s\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::ImuStreamInfo_Enabled:
        ROS_INFO("IMU Streaming Enabled: \t\t\t\t%s\n", strcmp((it->second).c_str(),"1") == 0 ? "TRUE" : "FALSE");
        break;

      case mscl::DeviceStatusValues::ImuStreamInfo_PacketsDropped:
        ROS_INFO("Number of Dropped IMU Packets: \t\t\t%s Packets\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::EstimationFilterStreamInfo_Enabled:
        ROS_INFO("FILTER Streaming Enabled: \t\t\t%s\n", strcmp((it->second).c_str(),"1") == 0 ? "TRUE" : "FALSE");
        break;

      case mscl::DeviceStatusValues::EstimationFilterStreamInfo_PacketsDropped:
        ROS_INFO("Number of Dropped FILTER Packets: \t\t%s Packets\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::ComPortInfo_BytesWritten:
        ROS_INFO("Communications Port Bytes Written: \t\t%s Bytes\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::ComPortInfo_BytesRead:
        ROS_INFO("Communications Port Bytes Read: \t\t%s Bytes\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::ComPortInfo_OverrunsOnWrite:
        ROS_INFO("Communications Port Write Overruns: \t\t%s\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::ComPortInfo_OverrunsOnRead:
        ROS_INFO("Communications Port Read Overruns: \t\t%s\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::ImuMessageInfo_MessageParsingErrors:
        ROS_INFO("IMU Parser Errors: \t\t\t\t%s Errors\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::ImuMessageInfo_MessagesRead:
        ROS_INFO("IMU Message Count: \t\t\t\t%s Messages\n", (it->second).c_str());
        break;

      case mscl::DeviceStatusValues::ImuMessageInfo_LastMessageReadinMS:
        ROS_INFO("IMU Last Message Received: \t\t\t%s ms\n", (it->second).c_str());
        break;

      default:
        break;
      }
    }
  }
  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Auto Angular Zupt Enable/Threshold Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

// Set zero angular-rate update threshold
bool Microstrain::set_zero_angle_update_threshold(ros_mscl::SetZeroAngleUpdateThreshold::Request &req,
                                                  ros_mscl::SetZeroAngleUpdateThreshold::Response &res)
{
  res.success = false;

  ROS_INFO("Setting Zero Angular-Rate-Update threshold\n");

  if(m_inertial_device)
  {
    try
    {
      mscl::ZUPTSettingsData ZUPTSettings(req.enable, req.threshold);
      m_inertial_device->setAngularRateZUPT(ZUPTSettings);

      ZUPTSettings = m_inertial_device->getAngularRateZUPT();
      ROS_INFO("Enable value set to: %d, Threshold is: %f rad/s",
               ZUPTSettings.enabled, ZUPTSettings.threshold);

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Auto Angular Zupt Enable/Threshold Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_zero_angle_update_threshold(ros_mscl::GetZeroAngleUpdateThreshold::Request &req,
                                                  ros_mscl::GetZeroAngleUpdateThreshold::Response &res)
{
  res.success = false;

  ROS_INFO("Getting Zero Angular-Rate-Update threshold\n");

  if(m_inertial_device)
  {
    try
    {
      mscl::ZUPTSettingsData ZUPTSettings = m_inertial_device->getAngularRateZUPT();
      ROS_INFO("Enable value set to: %d, Threshold is: %f rad/s",
               ZUPTSettings.enabled, ZUPTSettings.threshold);

      res.enable    = ZUPTSettings.enabled;
      res.threshold = ZUPTSettings.threshold;
      res.success   = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Auto Velocity Zupt Enable/Threshold Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_zero_velocity_update_threshold(ros_mscl::SetZeroVelocityUpdateThreshold::Request &req,
                                                     ros_mscl::SetZeroVelocityUpdateThreshold::Response &res)
{
  res.success = false;

  ROS_INFO("Setting Zero Velocity-Update threshold\n");

  if(m_inertial_device)
  {
    try
    {
      mscl::ZUPTSettingsData ZUPTSettings(req.enable, req.threshold);
      m_inertial_device->setVelocityZUPT(ZUPTSettings);

      ZUPTSettings = m_inertial_device->getVelocityZUPT();
      ROS_INFO("Enable value set to: %d, Threshold is: %f m/s",
               ZUPTSettings.enabled, ZUPTSettings.threshold);

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Auto Velocity Zupt Enable/Threshold Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_zero_velocity_update_threshold(ros_mscl::GetZeroVelocityUpdateThreshold::Request &req,
                                                     ros_mscl::GetZeroVelocityUpdateThreshold::Response &res)
{
  res.success = false;

  ROS_INFO("Getting Zero Velocity-Update threshold\n");

  if(m_inertial_device)
  {
    try
    {
      mscl::ZUPTSettingsData ZUPTSettings = m_inertial_device->getVelocityZUPT();
      ROS_INFO("Enable value set to: %d, Threshold is: %f rad/s",
               ZUPTSettings.enabled, ZUPTSettings.threshold);

      res.enable    = ZUPTSettings.enabled;
      res.threshold = ZUPTSettings.threshold;
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Tare Orientation Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_tare_orientation(ros_mscl::SetTareOrientation::Request &req,
                                       ros_mscl::SetTareOrientation::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      mscl::TareAxisValues axisValue(req.axis & mscl::InertialTypes::TARE_PITCH_AXIS,
                                     req.axis & mscl::InertialTypes::TARE_ROLL_AXIS,
                                     req.axis & mscl::InertialTypes::TARE_YAW_AXIS);
      m_inertial_device->tareOrientation(axisValue);

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Accel Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_accel_noise(ros_mscl::SetAccelNoise::Request &req,
                                  ros_mscl::SetAccelNoise::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Setting the accel noise values\n");
      mscl::GeometricVector noise(req.noise.x, req.noise.y, req.noise.z);
      m_inertial_device->setAccelNoiseStandardDeviation(noise);

      noise = m_inertial_device->getAccelNoiseStandardDeviation();
      ROS_INFO("Accel noise values successfully set.\n");
      ROS_INFO("Returned values: %f X %f Y %f Z\n",
               noise.x(), noise.y(), noise.z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Accel Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_accel_noise(ros_mscl::GetAccelNoise::Request &req,
                                  ros_mscl::GetAccelNoise::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Getting the accel noise values\n");

      mscl::GeometricVector noise = m_inertial_device->getAccelNoiseStandardDeviation();
      ROS_INFO("Returned values: %f X %f Y %f Z\n",
               noise.x(), noise.y(), noise.z());

      res.noise.x = noise.x();
      res.noise.y = noise.y();
      res.noise.z = noise.z();
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Gyro Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_gyro_noise(ros_mscl::SetGyroNoise::Request &req,
                                 ros_mscl::SetGyroNoise::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Setting the gyro noise values\n");
      mscl::GeometricVector noise(req.noise.x, req.noise.y, req.noise.z);
      m_inertial_device->setGyroNoiseStandardDeviation(noise);

      noise = m_inertial_device->getGyroNoiseStandardDeviation();
      ROS_INFO("Gyro noise values successfully set.\n");
      ROS_INFO("Returned values: %f X %f Y %f Z\n",
               noise.x(), noise.y(), noise.z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Gyro Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_gyro_noise(ros_mscl::GetGyroNoise::Request &req,
                                 ros_mscl::GetGyroNoise::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Getting the gyro noise values\n");

      mscl::GeometricVector noise = m_inertial_device->getGyroNoiseStandardDeviation();
      ROS_INFO("Gyro noise values: %f X %f Y %f Z\n",
               noise.x(), noise.y(), noise.z());

      res.noise.x = noise.x();
      res.noise.y = noise.y();
      res.noise.z = noise.z();
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Magnetometer Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_mag_noise(ros_mscl::SetMagNoise::Request &req,
                                ros_mscl::SetMagNoise::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Setting the mag noise values\n");
      mscl::GeometricVector noise(req.noise.x, req.noise.y, req.noise.z);
      m_inertial_device->setHardIronOffsetProcessNoise(noise);

      noise = m_inertial_device->getHardIronOffsetProcessNoise();
      ROS_INFO("Mag noise values successfully set.\n");
      ROS_INFO("Returned values: %f X %f Y %f Z\n",
               noise.x(), noise.y(), noise.z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Magnetometer Noise 1-Sigma Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_mag_noise(ros_mscl::GetMagNoise::Request &req,
                                ros_mscl::GetMagNoise::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Getting the mag noise values\n");
      mscl::GeometricVector noise = m_inertial_device->getHardIronOffsetProcessNoise();
      ROS_INFO("Returned values: %f X %f Y %f Z\n",
               noise.x(), noise.y(), noise.z());

      res.noise.x = noise.x();
      res.noise.y = noise.y();
      res.noise.z = noise.z();
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Gyro Bias Model Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_gyro_bias_model(ros_mscl::SetGyroBiasModel::Request &req,
                                      ros_mscl::SetGyroBiasModel::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Setting the gyro bias model values\n");

      mscl::GeometricVectors collection;
      mscl::GeometricVector noise(req.noise_vector.x, req.noise_vector.y, req.noise_vector.z);
      collection.push_back(noise);

      mscl::GeometricVector beta_vector(req.beta_vector.x, req.beta_vector.y, req.beta_vector.z);
      collection.push_back(beta_vector);

      m_inertial_device->setGyroBiasModelParams(collection);

      collection = m_inertial_device->getGyroBiasModelParams();
      ROS_INFO("Gyro bias model values successfully set.\n");
      ROS_INFO("Returned values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n",
               collection[0].x(), collection[0].y(), collection[0].z(),
               collection[1].x(), collection[1].y(), collection[1].z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Gyro Bias Model Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_gyro_bias_model(ros_mscl::GetGyroBiasModel::Request &req,
                                      ros_mscl::GetGyroBiasModel::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Getting the gyro bias model values\n");
      mscl::GeometricVectors collection = m_inertial_device->getGyroBiasModelParams();
      ROS_INFO("Gyro bias model values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n",
               collection[0].x(), collection[0].y(), collection[0].z(),
               collection[1].x(), collection[1].y(), collection[1].z());

      res.noise_vector.x = collection[0].x();
      res.noise_vector.y = collection[0].y();
      res.noise_vector.z = collection[0].z();
      res.beta_vector.x  = collection[1].x();
      res.beta_vector.y  = collection[1].y();
      res.beta_vector.z  = collection[1].z();

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Accel Bias Model Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_accel_bias_model(ros_mscl::SetAccelBiasModel::Request &req,
                                       ros_mscl::SetAccelBiasModel::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Setting the accel bias model values\n");
      mscl::GeometricVectors collection;
      mscl::GeometricVector noise(req.noise_vector.x, req.noise_vector.y, req.noise_vector.z);
      collection.push_back(noise);

      mscl::GeometricVector beta_vector(req.beta_vector.x, req.beta_vector.y, req.beta_vector.z);
      collection.push_back(beta_vector);
      m_inertial_device->setAccelBiasModelParams(collection);

      collection = m_inertial_device->getAccelBiasModelParams();
      ROS_INFO("Accel bias model values successfully set.\n");
      ROS_INFO("Returned values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n",
               collection[0].x(), collection[0].y(), collection[0].z(),
               collection[1].x(), collection[1].y(), collection[1].z());

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Accel Bias Model Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_accel_bias_model(ros_mscl::GetAccelBiasModel::Request &req,
                                       ros_mscl::GetAccelBiasModel::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Getting the accel bias model values\n");
      mscl::GeometricVectors collection = m_inertial_device->getAccelBiasModelParams();

      ROS_INFO("Accel bias model values:  Beta: %f X %f Y %f Z, White Noise: %f X %f Y %f Z\n",
               collection[0].x(), collection[0].y(), collection[0].z(),
               collection[1].x(), collection[1].y(), collection[1].z());

      res.noise_vector.x = collection[0].x();
      res.noise_vector.y = collection[0].y();
      res.noise_vector.z = collection[0].z();
      res.beta_vector.x  = collection[1].x();
      res.beta_vector.y  = collection[1].y();
      res.beta_vector.z  = collection[1].z();

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Accel Magnitude Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_gravity_adaptive_vals(ros_mscl::SetGravityAdaptiveVals::Request &req,
                                            ros_mscl::SetGravityAdaptiveVals::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Setting the accel magnitude error adaptive measurement values\n");

      mscl::AdaptiveMeasurementData adaptiveData;
      adaptiveData.mode                 = static_cast<mscl::InertialTypes::AdaptiveMeasurementMode>(req.enable);
      adaptiveData.lowPassFilterCutoff  = req.low_pass_cutoff;
      adaptiveData.lowLimit             = req.low_limit;
      adaptiveData.highLimit            = req.high_limit;
      adaptiveData.lowLimitUncertainty  = req.low_limit_1sigma;
      adaptiveData.highLimitUncertainty = req.high_limit_1sigma;
      adaptiveData.minUncertainty       = req.min_1sigma;

      m_inertial_device->setGravityErrorAdaptiveMeasurement(adaptiveData);

      adaptiveData = m_inertial_device->getGravityErrorAdaptiveMeasurement();
      ROS_INFO("accel magnitude error adaptive measurement values successfully set.\n");
      ROS_INFO("Returned values: Enable: %i, Parameters: %f %f %f %f %f %f",
               adaptiveData.mode, adaptiveData.lowPassFilterCutoff,
               adaptiveData.minUncertainty, adaptiveData.lowLimit,
               adaptiveData.highLimit, adaptiveData.lowLimitUncertainty,
               adaptiveData.highLimitUncertainty);

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Accel Magnitude Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_gravity_adaptive_vals(ros_mscl::GetGravityAdaptiveVals::Request &req,
                                            ros_mscl::GetGravityAdaptiveVals::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Getting the accel magnitude error adaptive measurement values\n");

      mscl::AdaptiveMeasurementData adaptiveData = m_inertial_device->getGravityErrorAdaptiveMeasurement();
      ROS_INFO("Accel magnitude error adaptive measurement values are: Enable: %i, Parameters: %f %f %f %f %f %f",
               adaptiveData.mode, adaptiveData.lowPassFilterCutoff,
               adaptiveData.minUncertainty, adaptiveData.lowLimit,
               adaptiveData.highLimit, adaptiveData.lowLimitUncertainty,
               adaptiveData.highLimitUncertainty);

      res.enable            = adaptiveData.mode;
      res.low_pass_cutoff   = adaptiveData.lowPassFilterCutoff;
      res.low_limit         = adaptiveData.lowLimit;
      res.high_limit        = adaptiveData.highLimit;
      res.low_limit_1sigma  = adaptiveData.lowLimitUncertainty;
      res.high_limit_1sigma = adaptiveData.highLimitUncertainty;
      res.min_1sigma        = adaptiveData.minUncertainty;

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Magnetometer Magnitude Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_mag_adaptive_vals(ros_mscl::SetMagAdaptiveVals::Request &req,
                                        ros_mscl::SetMagAdaptiveVals::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Setting the mag magnitude error adaptive measurement values\n");

      mscl::AdaptiveMeasurementData adaptiveData;
      adaptiveData.mode                 = static_cast<mscl::InertialTypes::AdaptiveMeasurementMode>(req.enable);
      adaptiveData.lowPassFilterCutoff  = req.low_pass_cutoff;
      adaptiveData.lowLimit             = req.low_limit;
      adaptiveData.highLimit            = req.high_limit;
      adaptiveData.lowLimitUncertainty  = req.low_limit_1sigma;
      adaptiveData.highLimitUncertainty = req.high_limit_1sigma;
      adaptiveData.minUncertainty       = req.min_1sigma;

      m_inertial_device->setMagnetometerErrorAdaptiveMeasurement(adaptiveData);

      adaptiveData = m_inertial_device->getMagnetometerErrorAdaptiveMeasurement();
      ROS_INFO("mag magnitude error adaptive measurement values successfully set.\n");

      ROS_INFO("Returned values: Enable: %i, Parameters: %f %f %f %f %f %f",
               adaptiveData.mode, adaptiveData.lowPassFilterCutoff,
               adaptiveData.minUncertainty, adaptiveData.lowLimit,
               adaptiveData.highLimit, adaptiveData.lowLimitUncertainty,
               adaptiveData.highLimitUncertainty);

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Magnetometer Magnitude Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_mag_adaptive_vals(ros_mscl::GetMagAdaptiveVals::Request &req,
                                        ros_mscl::GetMagAdaptiveVals::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Getting the mag magnitude error adaptive measurement values\n");

      mscl::AdaptiveMeasurementData adaptiveData = m_inertial_device->getMagnetometerErrorAdaptiveMeasurement();
      ROS_INFO("Mag magnitude error adaptive measurement values are: Enable: %i, Parameters: %f %f %f %f %f %f",
               adaptiveData.mode, adaptiveData.lowPassFilterCutoff, adaptiveData.minUncertainty, adaptiveData.lowLimit,
               adaptiveData.highLimit, adaptiveData.lowLimitUncertainty, adaptiveData.highLimitUncertainty);

      res.enable            = adaptiveData.mode;
      res.low_pass_cutoff   = adaptiveData.lowPassFilterCutoff;
      res.low_limit         = adaptiveData.lowLimit;
      res.high_limit        = adaptiveData.highLimit;
      res.low_limit_1sigma  = adaptiveData.lowLimitUncertainty;
      res.high_limit_1sigma = adaptiveData.highLimitUncertainty;
      res.min_1sigma        = adaptiveData.minUncertainty;

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Magnetometer Dip Angle Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_mag_dip_adaptive_vals(ros_mscl::SetMagDipAdaptiveVals::Request &req,
                                            ros_mscl::SetMagDipAdaptiveVals::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Setting the mag dip angle error adaptive measurement values\n");

      mscl::AdaptiveMeasurementData adaptiveData;
      adaptiveData.mode                 = static_cast<mscl::InertialTypes::AdaptiveMeasurementMode>(req.enable);
      adaptiveData.lowPassFilterCutoff  = req.low_pass_cutoff;
      adaptiveData.highLimit            = req.high_limit;
      adaptiveData.highLimitUncertainty = req.high_limit_1sigma;
      adaptiveData.minUncertainty       = req.min_1sigma;

      m_inertial_device->setMagDipAngleErrorAdaptiveMeasurement(adaptiveData);

      adaptiveData = m_inertial_device->getMagDipAngleErrorAdaptiveMeasurement();
      ROS_INFO("mag dip angle error adaptive measurement values successfully set.\n");
      ROS_INFO("Returned values: Enable: %i, Parameters: %f %f %f %f\n",
               adaptiveData.mode, adaptiveData.lowPassFilterCutoff,
               adaptiveData.minUncertainty, adaptiveData.highLimit,
               adaptiveData.highLimitUncertainty);

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Magnetometer Dip Angle Error Adaptive Measurement Values Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_mag_dip_adaptive_vals(ros_mscl::GetMagDipAdaptiveVals::Request &req,
                                            ros_mscl::GetMagDipAdaptiveVals::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Getting the mag dip angle error adaptive measurement values\n");

      mscl::AdaptiveMeasurementData adaptiveData = m_inertial_device->getMagDipAngleErrorAdaptiveMeasurement();
      ROS_INFO("Returned values: Enable: %i, Parameters: %f %f %f %f %f %f",
               adaptiveData.mode, adaptiveData.lowPassFilterCutoff,
               adaptiveData.minUncertainty, adaptiveData.lowLimit,
               adaptiveData.highLimit, adaptiveData.lowLimitUncertainty,
               adaptiveData.highLimitUncertainty);

      res.enable            = adaptiveData.mode;
      res.low_pass_cutoff   = adaptiveData.lowPassFilterCutoff;
      res.high_limit        = adaptiveData.highLimit;
      res.high_limit_1sigma = adaptiveData.highLimitUncertainty;
      res.min_1sigma        = adaptiveData.minUncertainty;

      res.success = true;
    }
    catch (mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Vehicle Dynamics Mode Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_dynamics_mode(ros_mscl::SetDynamicsMode::Request &req,
                                    ros_mscl::SetDynamicsMode::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      ROS_INFO("Setting the vehicle dynamics mode\n");

      mscl::InertialTypes::VehicleModeType mode = static_cast<mscl::InertialTypes::VehicleModeType>(req.mode);
      m_inertial_device->setVehicleDynamicsMode(mode);

      mode = m_inertial_device->getVehicleDynamicsMode();

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Vehicle Dynamics Mode Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_dynamics_mode(ros_mscl::GetDynamicsMode::Request &req,
                                    ros_mscl::GetDynamicsMode::Response &res)
{
  res.success = false;

  ROS_INFO("Getting the vehicle dynamics mode\n");

  if(m_inertial_device)
  {
    try
    {
      mscl::InertialTypes::VehicleModeType mode = m_inertial_device->getVehicleDynamicsMode();
      ROS_INFO("Vehicle dynamics mode is: %d\n", mode);

      res.mode    = mode;
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Change Device Settings
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::device_settings(ros_mscl::DeviceSettings::Request &req, ros_mscl::DeviceSettings::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      switch(req.function_selector)
      {
        //Save
        case 3:
        {
          ROS_INFO("Processing device settings command with function selector = 3 (Save)\n");
          m_inertial_device->saveSettingsAsStartup();
        }break;

        //Load Saved Settings
        case 4:
        {
          ROS_INFO("Processing device settings command with function selector = 4 (Load Saved Settings)\n");
          m_inertial_device->loadStartupSettings();
        }break;

        //Load Default Settings
        case 5:
        {
          ROS_INFO("Processing device settings command with function selector = 5 (Load Defailt Settings)\n");
          m_inertial_device->loadFactoryDefaultSettings();
        }break;

        //Unsupported function selector
        default:
        {
          ROS_INFO("Error: Unsupported function selector for device settings command\n");
          return res.success;
        }break;
      }

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Commanded Velocity Zupt Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::commanded_vel_zupt(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = false;

  if(m_inertial_device && m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_VEL_UPDATE))
  {
    try
    {
      m_inertial_device->cmdedVelZUPT();
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Commanded Angular Rate Zupt Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::commanded_ang_rate_zupt(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  res.success = false;

  if(m_inertial_device && m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_EF_CMDED_ZERO_ANG_RATE_UPDATE))
  {
    try
    {
      m_inertial_device->cmdedAngRateZUPT();
      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// External Heading Update Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::external_heading_update(ros_mscl::ExternalHeadingUpdate::Request &req,
                                          ros_mscl::ExternalHeadingUpdate::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      mscl::HeadingData heading_data;

      heading_data.headingAngle            = req.heading_rad;
      heading_data.headingAngleUncertainty = req.heading_1sigma_rad;
      heading_data.heading                 = (mscl::HeadingData::HeadingType)req.heading_type;

      mscl::TimeUpdate timestamp(req.gps_tow, req.gps_week_number);

      if(req.use_time)
      {
        m_inertial_device->sendExternalHeadingUpdate(heading_data, timestamp);
        ROS_INFO("Sent External Heading update with timestamp.\n");
      }
      else
      {
        m_inertial_device->sendExternalHeadingUpdate(heading_data);
        ROS_INFO("Sent External Heading update.\n");
      }

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Set Relative Position Reference Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::set_relative_position_reference(ros_mscl::SetRelativePositionReference::Request &req,
                                                  ros_mscl::SetRelativePositionReference::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      mscl::PositionReferenceConfiguration ref;

      ref.position   = mscl::Position(req.position.x, req.position.y, req.position.z, static_cast<mscl::PositionVelocityReferenceFrame>(req.frame));
      ref.autoConfig = !((bool)req.source);

      m_inertial_device->setRelativePositionReference(ref);

      if(req.source == 0)
        ROS_INFO("Setting reference position to RTK base station (automatic)");
      else
        ROS_INFO("Setting reference position to: [%f, %f, %f], ref frame = %d", req.position.x, req.position.y, req.position.z, req.frame);

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Get Relative Position Reference Service
/////////////////////////////////////////////////////////////////////////////////////////////////////

bool Microstrain::get_relative_position_reference(ros_mscl::GetRelativePositionReference::Request &req,
                                                  ros_mscl::GetRelativePositionReference::Response &res)
{
  res.success = false;

  if(m_inertial_device)
  {
    try
    {
      mscl::PositionReferenceConfiguration ref = m_inertial_device->getRelativePositionReference();

      if(ref.autoConfig)
        ROS_INFO("Reference position is set to RTK base station (automatic)");
      else
        ROS_INFO("Reference position is: [%f, %f, %f], ref frame = %d", ref.position.x(), ref.position.y(), ref.position.z(), (int)ref.position.referenceFrame);

      res.source     = !ref.autoConfig;
      res.frame      = (int)ref.position.referenceFrame;
      res.position.x = ref.position.x();
      res.position.y = ref.position.y();
      res.position.z = ref.position.z();

      res.success = true;
    }
    catch(mscl::Error &e)
    {
      ROS_ERROR("Error: %s", e.what());
    }
  }

  return res.success;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Diagnostic information posted to the device status topic and diagnostic agreggator
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Microstrain::device_status_callback()
{
  if(!m_inertial_device)
  {
    return;
  }

  if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    if(m_inertial_device->features().supportedStatusSelectors().size() > 1)
    {
      mscl::DeviceStatusData statusData = m_inertial_device->getDiagnosticDeviceStatus();
      mscl::DeviceStatusMap  status     = statusData.asMap();

      m_device_status_msg.system_timer_ms = statusData.systemTimerInMS;

      mscl::DeviceStatusMap::iterator it;

      for(it = status.begin(); it != status.end(); it++)
      {
        switch (it->first)
        {
        case mscl::DeviceStatusValues::ModelNumber:
          m_device_status_msg.device_model = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::StatusStructure_Value:
          m_device_status_msg.status_selector = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::SystemState_Value:
          m_device_status_msg.system_state = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::ImuStreamInfo_Enabled:
          m_device_status_msg.imu_stream_enabled = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::GnssStreamInfo_Enabled:
          m_device_status_msg.gps_stream_enabled = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::ImuStreamInfo_PacketsDropped:
          m_device_status_msg.imu_dropped_packets = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::EstimationFilterStreamInfo_Enabled:
          m_device_status_msg.filter_stream_enabled = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::EstimationFilterStreamInfo_PacketsDropped:
          m_device_status_msg.filter_dropped_packets = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::ComPortInfo_BytesWritten:
          m_device_status_msg.com1_port_bytes_written = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::ComPortInfo_BytesRead:
          m_device_status_msg.com1_port_bytes_read = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::ComPortInfo_OverrunsOnWrite:
          m_device_status_msg.com1_port_write_overruns = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::ComPortInfo_OverrunsOnRead:
          m_device_status_msg.com1_port_read_overruns = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::ImuMessageInfo_MessageParsingErrors:
          m_device_status_msg.imu_parser_errors = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::ImuMessageInfo_MessagesRead:
          m_device_status_msg.imu_message_count = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::ImuMessageInfo_LastMessageReadinMS:
          m_device_status_msg.imu_last_message_ms = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::GnssStreamInfo_PacketsDropped:
          m_device_status_msg.gps_dropped_packets = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::GnssMessageInfo_MessageParsingErrors:
          m_device_status_msg.gps_parser_errors = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::GnssMessageInfo_MessagesRead:
          m_device_status_msg.gps_message_count = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::GnssMessageInfo_LastMessageReadinMS:
          m_device_status_msg.gps_last_message_ms = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::GnssPowerStateOn:
          m_device_status_msg.gps_power_on = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::gnss1PpsPulseInfo_Count:
          m_device_status_msg.num_gps_pps_triggers = atoi(it->second.c_str());
          break;

        case mscl::DeviceStatusValues::gnss1PpsPulseInfo_LastTimeinMS:
          m_device_status_msg.last_gps_pps_trigger_ms = atoi(it->second.c_str());
          break;

        default:
          break;
        }
      }

      m_device_status_pub.publish(m_device_status_msg);
    }
  }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
// Packet Statistics Print Function
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Microstrain::print_packet_stats()
{
  if(m_inertial_device)
  {
    return;
  }

  if(m_inertial_device->features().supportsCommand(mscl::MipTypes::Command::CMD_DEVICE_STATUS))
  {
    if(m_inertial_device->features().supportedStatusSelectors().size() > 1)
    {
      mscl::DeviceStatusData status = m_inertial_device->getDiagnosticDeviceStatus();

      m_imu_valid_packet_count          = status.imuMessageInfo().messagesRead;
      m_imu_checksum_error_packet_count = status.imuMessageInfo().messageParsingErrors;
      m_imu_timeout_packet_count        = status.imuStreamInfo().outgoingPacketsDropped;
      m_filter_timeout_packet_count     = status.estimationFilterStreamInfo().outgoingPacketsDropped;

      ROS_DEBUG_THROTTLE(1.0, "%u IMU (%u errors) Packets",
                           m_imu_valid_packet_count, m_imu_timeout_packet_count + m_imu_checksum_error_packet_count);

      m_gnss_checksum_error_packet_count[GNSS1_ID] = status.gnssMessageInfo().messageParsingErrors;
      m_gnss_valid_packet_count[GNSS1_ID]          = status.gnssMessageInfo().messagesRead;
      m_gnss_timeout_packet_count[GNSS1_ID]        = status.gnssStreamInfo().outgoingPacketsDropped;

      ROS_DEBUG_THROTTLE(1.0, "%u FILTER (%u errors)    %u IMU (%u errors)    %u GPS (%u errors) Packets",
                         m_filter_valid_packet_count,  m_filter_timeout_packet_count,
                         m_imu_valid_packet_count,     m_imu_timeout_packet_count + m_imu_checksum_error_packet_count,
                         m_gnss_valid_packet_count[GNSS1_ID], m_gnss_timeout_packet_count[GNSS1_ID] + m_gnss_checksum_error_packet_count[GNSS1_ID]);

            ROS_DEBUG_THROTTLE(1.0, "%u FILTER (%u errors)    %u IMU (%u errors) Packets",
                         m_filter_valid_packet_count, m_filter_timeout_packet_count,
                         m_imu_valid_packet_count,    m_imu_timeout_packet_count + m_imu_checksum_error_packet_count);
    }
  }
}

} // namespace Microstrain
