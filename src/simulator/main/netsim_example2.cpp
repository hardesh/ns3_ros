/*
 * This example configures and starts the ROS interface
 * without configuring and starting a network. The simulated
 * network will be configured and started by calling to ROS
 * services. All packets are of the type dccomms::DataLinkFrame
 * with crc16.
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "dccomms_netsim");
  auto packetBuilder =
      dccomms::CreateObject<dccomms::DataLinkFramePacketBuilder>(
          dccomms::DataLinkFrame::crc16);
  auto Log = cpplogging::CreateLogger("netsim_example2");

  auto sim = dccomms::CreateObject<ROSCommsSimulator>();
  sim->SetLogName("netsim");
  sim->LogToFile("netsim_log");

  sim->SetDefaultPacketBuilder(packetBuilder);

  // Print log messages in callbacks
  sim->SetTransmitPDUCb([Log](ROSCommsDevice *dev, PacketPtr pkt) {
    Log->Info("Dev '{}': transmitting PDU to address {}", dev->GetDccommsId(),
              pkt->GetDestAddr());
  });
  sim->SetReceivePDUCb([Log](ROSCommsDevice *dev, PacketPtr pkt) {
    if (pkt->PacketIsOk())
      Log->Info("Dev '{}': received PDU from address {}", dev->GetDccommsId(),
                pkt->GetSrcAddr());
    else
      Log->Warn("Dev '{}': received PDU with errors", dev->GetDccommsId());

  });
  int printPositionPeriod = 50;
  sim->SetPositionUpdatedCb(
      [Log](ROSCommsDeviceNs3Ptr dev, tf::Vector3 pos) {
        Log->Debug("Dev '{}': P: [{},{},{}]", dev->GetDccommsId(), pos.x(),
                   pos.y(), pos.z());

      },
      printPositionPeriod); // Print the position of each device at a minimum
                            // period of printPositionPeriod ms

  sim->StartROSInterface();
  Log->Info("ROS Interface started.");

  ros::Rate loop_rate(30);
  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();
  }
}
