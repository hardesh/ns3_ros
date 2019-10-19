#ifndef DCCOMMS_ROS_CUSTOM_COMMS_CHANNEL_H_
#define DCCOMMS_ROS_CUSTOM_COMMS_CHANNEL_H_

#include <dccomms_ros/simulator/CommsChannel.h>
#include <dccomms_ros/simulator/CustomROSCommsDevice.h>
#include <dccomms_ros_msgs/types.h>
#include <list>

using namespace std;
namespace dccomms_ros {

class CustomCommsChannel : public CommsChannel {
public:
  CustomCommsChannel(uint32_t id);
  void SetMinPrTime(double prTime);
  void SetPrTimeInc(double inc);
  void AddDevice(CustomROSCommsDeviceNs3Ptr dev);
  void SendPacket(CustomROSCommsDeviceNs3Ptr dev, ns3PacketPtr pkt);
  uint32_t GetId() { return _rosChannelId; }
  CHANNEL_TYPE GetType() { return CUSTOM_CHANNEL; }

  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static ns3::TypeId GetTypeId(void);

private:
  uint32_t _rosChannelId;
  uint64_t _prTimeIncPerMeter, _minPrTime; // nanoseconds
  std::list<CustomROSCommsDeviceNs3Ptr> _devices;
};

typedef ns3::Ptr<CustomCommsChannel> CustomCommsChannelNs3Ptr;
}

#endif // COMMSCHANNELPROPERTIES_H
