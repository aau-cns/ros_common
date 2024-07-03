/******************************************************************************
 * FILENAME:     INode.hpp
 * PURPOSE:      Automatically binds the dynamic reconfig server with the concrete callback.
 *               The user needs to implement update_(...) which updates the internal states.
 * AUTHOR:       jungr
 * MAIL:         roland.jung@ieee.org
 * VERSION:      v0.0.1
 * LICENCE       MIT
 * CREATION:     19.10.2018
 *
 *  Copyright (C) 2018 Roland Jung, Control of Networked Systems, University of Klagenfurt, Austria.
 *  All rights reserved. See the LICENSE file for details.
 ******************************************************************************/

#ifndef ROS_COMMON_ITROSDYNAMIC_HPP
#define ROS_COMMON_ITROSDYNAMIC_HPP

#include <dynamic_reconfigure/server.h>

namespace ros_common
{

  /**
     * abstract template class
     * Starts a dynamic reconfigure server for the configuration T.
     * On a change the callback will store the latest value in mDynConfig
     * and call the abstract update_() method in order to notify the child class
     */
  template < typename T>
  class ITROSdynamic
  {
    public:

      ITROSdynamic()
      {
        mFunc = boost::bind(&ITROSdynamic::callback, this, _1, _2);
        mServer.setCallback(mFunc);
      }

      void callback(T &config, uint32_t level)
      {
        ROS_DEBUG_STREAM("ITROSdynamic::callback(): reconfigure request receiver at level=" + std::to_string(level));
        mDynConfig = config;  //read
        if(!update_() )       // modify
        {
          ROS_WARN("ITROSdynamic::callback(): config update failed!");
        }
        config = mDynConfig;  // write
      }

      virtual bool update_()
      {
        return true;
      }


    protected:
      dynamic_reconfigure::Server<T> mServer;
      typename dynamic_reconfigure::Server<T>::CallbackType mFunc;

      T mDynConfig;

  }; // class ITROSdynamic

} // namespace ros_common


#endif // ROS_COMMON_ITROSDYNAMIC_HPP
