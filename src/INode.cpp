/******************************************************************************
 * FILENAME:     INode.cpp
 * PURPOSE:      Abstract baseclass for a node with custom spinning order of execution of virtual methods.
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * LICENCE       MIT
 * CREATION:     14.12.2018
 *
 *  Copyright (C) 2018 Roland Jung, Control of Networked Systems, University of Klagenfurt, Austria.
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/
#include "../include/ros_common/INode.hpp"


void ros_common::INode::run()
{
  bool asyncCallbacks;  // s
  mNh.param<bool>("async_callbacks", asyncCallbacks, false);

  std::shared_ptr<ros::AsyncSpinner> pSpinner;
  if(asyncCallbacks)
  {
    pSpinner.reset(new ros::AsyncSpinner(0));
    pSpinner->start();
  }

  init_topics();  // 1
  print_topic_info();
  load_config();  // 2

  // profile process() function and overheader
  bool tmp;
  mNh.param<bool>("node_profile", tmp, true);
  mbProfile = tmp;

  ros::Time processTimeStop;
  ros::Time processTimeStart;

  // callback spinning rate
  double spinningRate;  // Hz
  mNh.param<double>("node_spinning_rate", spinningRate, 50.0);

  // warning message
  double warnAfter_s;  // s
  mNh.param<double>("node_warn_time", warnAfter_s, 15.0);



  double emptySpinCnt = 0;
  double warningEmptySpin = spinningRate * warnAfter_s;

  if(warningEmptySpin == 0)
  {
    warningEmptySpin = std::numeric_limits<double>::max();
  }

  ros::Rate    rate(spinningRate);

  while (ros::ok() && !mbStopProcess) {

    if(!asyncCallbacks)
    {
      // ref: http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
      // will call all the callbacks waiting to be called at that point in time.

      ros::spinOnce();
    }
    if(mbHasNewMessage || mbSpinningProcess)
    {
      if(!mbIsInitialized)
      {
        ROS_INFO("Initialize");
        init();  // 3
        ROS_INFO("Initialization done!");
        mbIsInitialized = true;
      }

      processTimeStart = ros::Time::now();
      ros::Duration overheadTime = processTimeStart - processTimeStop;
      process();  // 4
      processTimeStop = ros::Time::now();
      ros::Duration processTime = processTimeStop - processTimeStart;
      mPeriod_s    = (processTime + overheadTime).toSec();

      if(mbProfile)
      {

        double        process_hz  = (processTime.toSec() > 0) ? 1 / processTime.toSec() : 0;
        double        period_hz   = (mPeriod_s) ? 1 / mPeriod_s : 0;
        ROS_INFO_THROTTLE(1, "process() %fs (%f Hz), overhead %fs, (%f Hz)", processTime.toSec(), process_hz,
                 overheadTime.toSec(), period_hz);
      }

      mbHasNewMessage = false; // message is processed
      emptySpinCnt   = 0;
    } else if (!mbSpinningProcess) {
      emptySpinCnt++;

      if(emptySpinCnt >= warningEmptySpin)
      {
        ROS_WARN("No message received in the last %f s", warnAfter_s);
        emptySpinCnt = 0;
      }
    }

    rate.sleep();
  }

  if(asyncCallbacks)
  {
    pSpinner->stop();
  }

}


void ros_common::INode::get_param_u32(const std::string &key, std::uint32_t &value, const std::uint32_t &def)
{
  int _val;
  mNh.param<int>(key, _val, def);
  value = static_cast<std::uint32_t>(_val);
}

void ros_common::INode::get_param_s16(const std::string &key, std::int16_t &value, const std::int16_t &def)
{
  int _val;
  mNh.param<int>(key, _val, def);
  value = static_cast<std::int16_t>(_val);
}

void ros_common::INode::param_get_string(std::string &variable, const std::string &param, const std::string &default_value)
{
  XmlRpc::XmlRpcValue xmlValue;
  mNh.getParam(param, xmlValue);

  if(xmlValue.getType() != XmlRpc::XmlRpcValue::TypeString)
  {
    ROS_WARN("Parameter %s invalid or not found! Set to default value %s", param.c_str(),
             default_value.c_str());
    variable = default_value;
  }
  else
  {
    variable = static_cast<std::string>(xmlValue);
    ROS_INFO("%s value loaded from config file: %s", param.c_str(), variable.c_str());
  }
}

void ros_common::INode::param_get_int(int &variable, const std::string &param, const int &default_value)
{
  XmlRpc::XmlRpcValue xmlValue;
  mNh.getParam(param, xmlValue);

  if(xmlValue.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    ROS_WARN("Parameter %s invalid or not found! Set to default value %d", param.c_str(), default_value);
    variable = default_value;
  }
  else
  {
    variable = static_cast<int>(xmlValue);
    ROS_INFO("%s value loaded from config file: %d", param.c_str(), variable);
  }
}

void ros_common::INode::param_get_uint(unsigned &variable, const std::string &param, const unsigned &default_value)
{
  XmlRpc::XmlRpcValue xmlValue;
  mNh.getParam(param, xmlValue);

  if(xmlValue.getType() != XmlRpc::XmlRpcValue::TypeInt)
  {
    ROS_WARN("Parameter %s invalid or not found! Set to default value %d", param.c_str(), default_value);
    variable = default_value;
  }
  else
  {
    variable = (unsigned)static_cast<int>(xmlValue);
    ROS_INFO("%s value loaded from config file: %d", param.c_str(), variable);
  }
}

void ros_common::INode::param_get_float(float &variable, const std::string &param, const float &default_value)
{
  XmlRpc::XmlRpcValue xmlValue;
  mNh.getParam(param, xmlValue);

  if(xmlValue.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    ROS_WARN("Parameter %s invalid or not found! Set to default value %f", param.c_str(), default_value);
    variable = default_value;
  }
  else
  {
    variable = (float) static_cast<double>(xmlValue);
    ROS_INFO("%s value loaded from config file: %f", param.c_str(), variable);
  }
}

void ros_common::INode::print_topic_info()
{
  //  print published/subscribed topics
  std::string   nodeName = ros::this_node::getName();
  ros::V_string topics;
  ros::this_node::getSubscribedTopics(topics);
  std::string   topicsStr = "\n\tnode name: " + nodeName + ":\n";

  topicsStr += "\tsubscribed to topics:\n";
  for (unsigned int i         = 0; i < topics.size(); i++)
  {
    topicsStr += ("\t\t" + topics.at(i) + "\n");
  }

  topicsStr += "\tadvertised topics:\n";
  ros::this_node::getAdvertisedTopics(topics);

  for (unsigned int i = 0; i < topics.size(); i++)
  {
    topicsStr += ("\t\t" + topics.at(i) + "\n");
  }

  ROS_INFO_STREAM("" << topicsStr);
}
