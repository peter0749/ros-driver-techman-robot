/*
 * Copyright 2019 Liang-Chi Hsieh
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include "tm_driver/tm_driver.h"
#include <gripper_test_node.h>

bool isClose(const double a, const double b)
{
    if (abs(a-b) > 0.005) return false;
    return true;
}

void getDI(TmDriver* robot, std::vector<bool>& vec) {
  robot->interface->stateRT->getDigitalInputEE(vec);
}

void getDO(TmDriver* robot, std::vector<bool>& vec) {
  robot->interface->stateRT->getDigitalOutputEE(vec);
}

/**
 * This method closes gripper, if it is in open or hold status.
 */
void closeGripper(TmDriver* robot) {

  std::vector<bool> vec = {false, false, false, false};

  getDI(robot, vec);
  bool gripperOpen = vec[0] == true && vec[1] == false && vec[2] == false;
  bool gripperHold = vec[0] == false && vec[1] == false && vec[2] == true;
  if (gripperOpen || gripperHold) {
    // Set DO1 to off.
    robot->setDigitalOutputEE(1, false);
    // Set DO0 to on.
    robot->setDigitalOutputEE(0, true);

    // Read input
    getDI(robot, vec);
    while (1) {
      if (vec[0] == false && vec[1] == true && vec[2] == false) {
        print_info("Gripper is closed.");
        break;
      } else if (vec[0] == false && vec[1] == false && vec[2] == true) {
        print_info("Gripper is hold when closing.");
        break;
      }
      getDI(robot, vec);
    }
  } else {
    print_info("Gripper is not in open or hold status. Can't close it.");
  }
}

/**
 * This method opens gripper, if it is in close or hold status.
 */
void openGripper(TmDriver* robot) {
  std::vector<bool> vec = {false, false, false, false};

  getDI(robot, vec);
  bool gripperClosed = vec[0] == false && vec[1] == true && vec[2] == false;
  bool gripperHold = vec[0] == false && vec[1] == false && vec[2] == true;
  if (gripperClosed || gripperHold) {
    // Set DO1 to off.
    robot->setDigitalOutputEE(1, false);
    // Set DO0 to off.
    robot->setDigitalOutputEE(0, false);

    // Read input
    getDI(robot, vec);
    while (1) {
      if (vec[0] == true && vec[1] == false && vec[2] == false) {
        print_info("Gripper is open.");
        break;
      } else if (vec[0] == false && vec[1] == false && vec[2] == true) {
        print_info("Gripper is hold when opening.");
        break;
      }
      getDI(robot, vec);
    }
  } else {
    print_info("Gripper is not in close or hold status. Can't open it.");
  }
}

/**
 * This methods sets fixed position of the gripper at close direction.
 */
void setClosePosition(TmDriver* robot) {
  std::vector<bool> vec = {false, false, false, false};

  getDO(robot, vec);
  if (vec[0] == false) {
    print_info("Gripper is now open. Going to close it...");

    // Set DO1
    robot->setDigitalOutputEE(1, true);
    // Set DO0
    robot->setDigitalOutputEE(0, true);

    // Read input
    getDI(robot, vec);
    while (vec[0] != false && vec[2] != true) {
      print_info("DI0: %d, DI2: %d", vec[0], vec[2]);
      getDI(robot, vec);
    }

    robot->setDigitalOutputEE(1, false);

    getDI(robot, vec);
    while (vec[1] != true && vec[2] != true) {
      print_info("DI1: %d, DI2: %d", vec[1], vec[2]);
      getDI(robot, vec);
    }
    print_info("Gripper is closed.");
  } else {
    print_info("Gripper is not in open status. Release it first!");
  }  
}

/**
 * This methods sets fixed position of the gripper at open direction.
 */
void setOpenPosition(TmDriver* robot) {
  std::vector<bool> vec = {false, false, false, false};

  getDO(robot, vec);
  if (vec[0] == true) {
    print_info("Gripper is now closed. Going to open it...");

    // Set DO1
    robot->setDigitalOutputEE(1, true);
    // Set DO0
    robot->setDigitalOutputEE(0, false);

    // Read input
    getDI(robot, vec);
    while (vec[1] != false && vec[2] != true) {
      print_info("DI1: %d, DI2: %d", vec[1], vec[2]);
      getDI(robot, vec);
    }

    robot->setDigitalOutputEE(1, false);

    getDI(robot, vec);
    while (vec[0] != true && vec[2] != true) {
      print_info("DI0: %d, DI2: %d", vec[1], vec[2]);
      getDI(robot, vec);
    }
    print_info("Gripper is released.");
  } else {
    print_info("Gripper is not in close status. Close it first!");
  }  
}

/**
 * This ROS node tests CHG2 gripper on TM5-700 robotic arm. It assumes pin between CHG2 and TM5-700:
 *   CHG2 open/close input <-> DO0
 *   CHG2 set input <-> DO1
 *   CHG2 open output <-> DI0
 *   CHG2 close output <-> DI1
 *   CHG2 hold output <-> DI2
 */

TMGripperInterface::TMGripperInterface(TmDriver* robot): node_handle_("") {

    finger_l_joint_pub = node_handle_.advertise < std_msgs::Float64 > ("/finger_l_controller/command", 10);
    finger_r_joint_pub = node_handle_.advertise < std_msgs::Float64 > ("/finger_r_controller/command", 10);

    gripper_open_sub_ = node_handle_.subscribe("gripper/open", 10, & TMGripperInterface::gripperOpen, this);
    gripper_close_sub_ = node_handle_.subscribe("gripper/close", 10, & TMGripperInterface::gripperClose, this);

    gripper_state_pub = node_handle_.advertise<std_msgs::Int8>("gripper/state", 10);

    gripper_state = -1; // unknown

    this->robot = robot;

}

void TMGripperInterface::gripperClose(const std_msgs::Empty & mg) {
    gripper_state = 1; // closing
    closeGripper(robot);
}

void TMGripperInterface::gripperOpen(const std_msgs::Empty & mg) {
    gripper_state = -1; // unknown
    openGripper(robot);
}

void TMGripperInterface::gripperStateCallback(const ros::TimerEvent &) {
    std_msgs::Int8 gripper_state_msg;
    gripper_state_msg.data = gripper_state;
    gripper_state_pub.publish(gripper_state_msg);
}

void TMGripperInterface::recordArmJoints(const sensor_msgs::JointState::ConstPtr & msg) {
    for (int index = 0; index < NUMBER_OF_ARM_JOINTS; ++index)
        arm_state[index] = msg->position.at(index);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gripper test");
  ros::NodeHandle nh;

  std::string host;
  std::condition_variable data_cv;
  std::condition_variable data_cv_rt;

  if (!(ros::param::get("~robot_ip_address", host))) {
    if (argc > 1) {
      host = argv[1];
    } else {
      exit(1);
    }
  }
  ros::Rate loop_rate(10);

  print_info("Connecting to the robot...");

  TmDriver* robot = new TmDriver(data_cv, data_cv_rt, host, 0);

  bool isRobotStarted = robot->interface->start();

  if (!isRobotStarted) {
      print_info("Can't connect to TM robot: %s", host.c_str());
      return 0;
  }
  print_info("TM5_700 gripper test");

  TMGripperInterface *gripper_interface = new TMGripperInterface(robot);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  delete robot;
  robot = NULL;
  
  delete gripper_interface;
  gripper_interface = NULL;

  return 0;
}
