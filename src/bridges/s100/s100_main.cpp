/*
 * main.cpp
 *
 *  Created on: 22 oct. 2016
 *      Author: Diego Centelles Beltran
 */

#include <cstdio>
#include <dccomms/CommsBridge.h>
#include <dccomms/DataLinkFrame.h>
#include <dccomms/Utils.h>
#include <dccomms_utils/S100Stream.h>
#include <iostream>

#include <cstdio>
#include <signal.h>
#include <sys/types.h>

// ROS
#include "ros/ros.h"
// end ROS

#include <cpplogging/cpplogging.h>

using namespace std;
using namespace dccomms;
using namespace dccomms_utils;
using namespace cpplogging;
CommsBridge *comms;
S100Stream *stream;
LoggerPtr Log;

void SIGINT_handler(int sig) {
  printf("Received %d signal\nClosing device socket...\n", sig);
  printf("Device closed.\n");
  fflush(stdout);
  comms->FlushLog();
  stream->FlushLog();
  Utils::Sleep(2000);
  printf("Log messages flushed.\n");

  exit(0);
}

void setSignals() {
  if (signal(SIGINT, SIGINT_handler) == SIG_ERR) {
    printf("SIGINT install error\n");
    exit(1);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dccomms_S100_bridge");
  ros::NodeHandle nh("~");

  auto logLevel = info;

  Log = CreateLogger("S100Bridge");
  Log->SetLogName(Log->GetLogName() + ":Main");
  Log->SetLogLevel(logLevel);

  std::string logPrefix;
  bool logToFileEnabled;
  if (!nh.getParam("logPrefix", logPrefix)) {
    Log->Error("Failed to get param 'logPrefix'");
    return 1;
  } else {
    if (logPrefix == "") {
      Log->Info("Do not log to file");
      logToFileEnabled = false;
    } else {
      Log->Info("Log prefix: {}", logPrefix);
      logToFileEnabled = true;
    }
  }

  if (logToFileEnabled) {
    Log->LogToFile(logPrefix + "_main");
  }

  std::string modemPort;
  if (!nh.getParam("modemPort", modemPort)) {
    Log->Error("Failed to get param 'modemPort'");
    return 1;
  } else {
    Log->Info("modem port: {}", modemPort);
  }

  int modemBaudrate;
  if (!nh.getParam("modemBaudrate", modemBaudrate)) {
    Log->Error("Failed to get param 'modemBaudrate'");
    return 1;
  } else {
    Log->Info("modem baudrate: {}", modemBaudrate);
  }

  std::string dccommsId;
  if (!nh.getParam("ns", dccommsId)) {
    Log->Error("Failed to get param 'dccommsId'");
    return 1;
  } else {
    Log->Info("dccommsId: {}", dccommsId);
  }

  setSignals();

  auto portBaudrate = SerialPortStream::BAUD_2400;
  stream = new S100Stream(modemPort, portBaudrate, modemBaudrate);
  PacketBuilderPtr pb =
      CreateObject<DataLinkFramePacketBuilder>(DataLinkFrame::fcsType::crc16);

  comms = new CommsBridge(stream, pb, pb, 0);

  comms->SetLogLevel(logLevel);
  stream->SetLogLevel(logLevel);
  comms->FlushLogOn(logLevel);
  stream->FlushLogOn(logLevel);

  comms->SetCommsDeviceId(dccommsId);
  comms->SetLogName("S100Bridge");
  stream->SetLogName(comms->GetLogName() + ":S100Stream");

  if (logToFileEnabled) {

    comms->LogToFile(logPrefix);
    stream->LogToFile(logPrefix + "_device");
  }
  stream->LogConfig();

  auto txtr = CreateObject<TransportPDU>();
  auto txpkt = comms->GetTxPacket();
  txtr->SetBuffer(txpkt->GetPayloadBuffer());

  comms->SetTransmitingPacketCb([txtr](const PacketPtr pkt) {
    Log->Info("(Seq: {}) Transmitting packet!", txtr->GetSeqNum());
  });

  auto rxtr = CreateObject<TransportPDU>();
  auto rxpkt = comms->GetRxPacket();
  rxtr->SetBuffer(rxpkt->GetPayloadBuffer());

  comms->SetReceivedPacketWithErrorsCb([rxtr](const PacketPtr pkt) {
    Log->Warn("(Seq: {}) Received packet with errors!", rxtr->GetSeqNum());
  });

  comms->SetReceivedPacketWithoutErrorsCb([rxtr](const PacketPtr pkt) {
    Log->Info("(Seq: {}) Received packet without errors!", rxtr->GetSeqNum());
  });

  comms->Start();
  while (1) {
    Utils::Sleep(10000);
  }
}
