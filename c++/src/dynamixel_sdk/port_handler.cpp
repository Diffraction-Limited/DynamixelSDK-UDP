/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
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
*******************************************************************************/

/* Author: zerom, Ryu Woon Jung (Leon) */

#if defined(__linux__)
#include "port_handler.h"
#include "port_handler_linux.h"
#include "port_handler_udp.h"
#elif defined(__APPLE__)
#include "port_handler.h"
#include "port_handler_mac.h"
#elif defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#include "port_handler.h"
#include "port_handler_windows.h"
#include "port_handler_udp.h" 
#elif defined(ARDUINO) || defined(__OPENCR__) || defined(__OPENCM904__) || defined(ARDUINO_OpenRB)
#include "../../include/dynamixel_sdk/port_handler.h"
#include "../../include/dynamixel_sdk/port_handler_arduino.h"
#endif

using namespace dynamixel;

PortHandler *PortHandler::getPortHandler(const char *port_name)
{
#if defined(__linux__) || defined(_WIN32) || defined(_WIN64)
  // Attempt to create a UDP port if the port_name is a valid UDP address
  try
  {
    return (PortHandler *)(new PortHandlerUDP(port_name));
  }
  catch(const std::exception& e)
  {
    // Note: fall back to usual port handlers if the port_name is not a valid UDP address
  }
#endif

#if defined(__linux__)
  return (PortHandler *)(new PortHandlerLinux(port_name));
#elif defined(__APPLE__)
  return (PortHandler *)(new PortHandlerMac(port_name));
#elif defined(_WIN32) || defined(_WIN64)
  return (PortHandler *)(new PortHandlerWindows(port_name));
#elif defined(ARDUINO) || defined(__OPENCR__) || defined(__OPENCM904__) || defined(ARDUINO_OpenRB)
  return (PortHandler *)(new PortHandlerArduino(port_name));
#endif
}
