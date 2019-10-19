/*
 * This example configures a simple network
 * and then starts the simulation by code.
 * All packets are of the type dccomms::DataLinkFrame
 * with crc16
 */

#include <cstdio>
#include <cstdio>
#include <iostream>
#include <signal.h>
#include <sys/types.h>

#include <dccomms_ros/simulator/ROSCommsSimulator.h>

// ROS
#include <ros/ros.h>
// end ROS

using namespace dccomms;
using namespace dccomms_ros;
using namespace std;

static std::shared_ptr<spd::logger> Log =
    spd::stdout_color_mt("CommsSimulatorTest");

int main(int argc, char **argv) {
  //// GET PARAMS
  ros::init(argc, argv, "dccomms_netsim");

  auto rovTxPacketBuilder =
      dccomms::CreateObject<dccomms::DataLinkFramePacketBuilder>(
          dccomms::DataLinkFrame::crc16);
  auto operatorTxPacketBuilder =
      dccomms::CreateObject<dccomms::DataLinkFramePacketBuilder>(
          dccomms::DataLinkFrame::crc16);

  auto sim = ns3::CreateObject<ROSCommsSimulator>();
  sim->SetLogName("netsim");
  sim->LogToFile("netsim_log");

  Log->set_level(spdlog::level::debug);
  Log->flush_on(spd::level::info);

  sim->SetPacketBuilder("rov", PACKET_TYPE::TX_PACKET, rovTxPacketBuilder);
  sim->SetPacketBuilder("rov", PACKET_TYPE::RX_PACKET, operatorTxPacketBuilder);

  sim->SetPacketBuilder("operator", PACKET_TYPE::TX_PACKET,
                        operatorTxPacketBuilder);
  sim->SetPacketBuilder("operator", PACKET_TYPE::RX_PACKET, rovTxPacketBuilder);

  dccomms_ros_msgs::AddCustomChannelRequest accreq;
  accreq.id = 0;
  accreq.minPrTime = 0;
  accreq.prTimeIncPerMeter = 0.66f;

  sim->AddCustomChannel(accreq);

  dccomms_ros_msgs::AddCustomDeviceRequest acdreq1;
  acdreq1.dccommsId = "rov";
  acdreq1.bitrate = 500;
  acdreq1.maxTxFifoSize = 512; // bytes
  acdreq1.mac = 1;
  acdreq1.maxDistance = 99999999;
  acdreq1.minDistance = 0;
  acdreq1.minPktErrorRate = 0;
  acdreq1.pktErrorRateIncPerMeter = 0.01;
  dccomms_ros_msgs::AddCustomDeviceRequest acdreq2 = acdreq1;
  acdreq2.dccommsId = "operator";
  acdreq2.mac = 2;
  sim->AddCustomDevice(acdreq1);
  sim->AddCustomDevice(acdreq2);

  dccomms_ros_msgs::LinkDeviceToChannelRequest ldc1;
  ldc1.channelId = 0;
  ldc1.dccommsId = "rov";

  dccomms_ros_msgs::LinkDeviceToChannelRequest ldc2;
  ldc2.channelId = 0;
  ldc2.dccommsId = "operator";

  sim->LinkDevToChannel(ldc1);
  sim->LinkDevToChannel(ldc2);

  ROSCommsDevice::PacketTransmittingCallback txcb =
      [](std::string path, ROSCommsDevice*, ns3PacketPtr) {
        Log->info("{}: Transmitting packet", path);
      };

  ROSCommsDevice::PacketReceivedCallback rxcb =
      [](std::string path, ROSCommsDevice*, ns3PacketPtr) {
        Log->info("{}: Packet received", path);
      };

  ns3::Config::Connect("/ROSDeviceList/*/PacketTransmitting",
                       ns3::MakeCallback(txcb));


  ns3::Config::Connect("/ROSDeviceList/*/PacketReceived",
                       ns3::MakeCallback(rxcb));

  sim->StartROSInterface();
  Log->info("ROS Interface started");

  sim->StartSimulation();
  Log->info("Simulation started");

  ros::Rate loop_rate(30);
  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();
  }
}
