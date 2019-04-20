/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <cstdio>

#include <llvm/raw_ostream.h>

#include "cscore.h"

#include <unistd.h>
#include <iostream>
#include <thread>

int main() {
  //  CS_Status s = 0;

        CS_Status s = 0;
        while (true) {
            for (auto&& i : cs::EnumerateUsbCameras(&s)) {
        std::cout << i.name << std::endl;
    }
    usleep(1000000);
}
    for (auto&& i : cs::EnumerateUsbCameras(&s)) {
        std::cout << i.name << std::endl;
    }
  llvm::outs() << "hostname: " << cs::GetHostname() << '\n';
  llvm::outs() << "IPv4 network addresses:\n";
  for (const auto& addr : cs::GetNetworkInterfaces())
    llvm::outs() << "  " << addr << '\n';
  cs::UsbCamera camera{"usbcam", 1};
  camera.SetVideoMode(cs::VideoMode::kMJPEG, 320, 240, 30);
  cs::MjpegServer mjpegServer{"httpserver", 8081};
  mjpegServer.SetSource(camera);

  while(true) {
      usleep(1000000);
  }

  std::getchar();
}