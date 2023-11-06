---
title: ros_common
author:
  - Roland Jung
  - Control of Networked Systems, University of Klagenfurt, Austria
date: 06.11.2023
subtitle: Version 1.0

documentclass: scrartcl
numbersections: true

toc: true
---

# ros_common

ROS package containing basic node/package features which are needed for the ROS example_node package

Maintainer: [@rojung](https://gitlab.aau.at/rojung)

![ros_common](/doc/ros_common.png)

## Getting Started

Tested for ROS Kinetic or later.

## Usage

Derive from abstract base class [INode](./include/ros_common/INode.hpp) if you need the following functionalty:
```
* abstract baseclass for node with custom spinning
* order of execution of virtual methods:
*  1) init_topics() subscribe and publish topics, initialize message independet stuff
*  1.1) print_topic_info()
*  2) load_config()
*  3) init() called after first message is arrived
*  4) process() called with each message
*
*  signal run() that new data is arrived with the mbHasNewMessage flag
*
*  params:
*   node_profile: enable timing mesauremnet for process() and overhead
*   node_spinning_rate: desired spinning rate [Hz]. default 50 Hz
*   node_warn_time: warn if no message arrived in X s. default 15 (0 do disable)
```

## Dependencies

- roscpp
- dynamic_reconfigure
