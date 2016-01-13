/*
 * Copyright (c) 2014-2015, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Control and Pattern Recognition Group,
 *       Warsaw University of Technology nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rtt/Component.hpp>
#include <string>
#include "eigen_conversions/eigen_msg.h"
#include "kdl_conversions/kdl_msg.h"
#include "Irp6Diagnostic_2.h"

Irp6Diagnostic_2::Irp6Diagnostic_2(const std::string& name)
    : RTT::TaskContext(name) {

  this->ports()->addPort("Diagnostics", port_Diagnostics).doc("");
  this->addProperty("hardware_label", hardware_label_).doc("");
}

Irp6Diagnostic_2::~Irp6Diagnostic_2() {
}

bool Irp6Diagnostic_2::configureHook() {
  char cCurrentPath[100];

      if (!getcwd(cCurrentPath, sizeof(cCurrentPath))) {
        return errno;
      }

      cCurrentPath[sizeof(cCurrentPath) - 1] = '\0';

      time_t now = time(0);
      struct tm newtime;
      char buf[80];
      localtime_r(&now, &newtime);
      strftime(buf, sizeof(buf), "_%Y-%m-%d_%X", &newtime);

      std::string filename = std::string(cCurrentPath);
      int ros = filename.find(".ros");
      filename = filename.erase(ros) + "log_" + std::string(buf);

      file.open((filename + ".csv").c_str());

  return true;
}

bool Irp6Diagnostic_2::startHook() {
  return true;
}

void Irp6Diagnostic_2::updateHook() {
  if (port_Diagnostics.read(diagnostic_) == RTT::NewData) {
    struct timeval tp;
    gettimeofday(&tp, NULL);
    int64_t ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    std::ostringstream output;
     output << diagnostic_.data << ";" << ms << ";\n";
     file << output.str();
  }
}

ORO_CREATE_COMPONENT(Irp6Diagnostic_2)

