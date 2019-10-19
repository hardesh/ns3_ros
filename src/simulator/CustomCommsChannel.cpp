#include <dccomms_ros/simulator/CustomCommsChannel.h>
#include <ns3/core-module.h>
#include <ns3/simulator.h>

using namespace ns3;

namespace dccomms_ros {

NS_OBJECT_ENSURE_REGISTERED(CustomCommsChannel);

ns3::TypeId CustomCommsChannel::GetTypeId(void) {
  static ns3::TypeId tid =
      ns3::TypeId("dccomms_ros::CustomCommsChannel").SetParent<CommsChannel>();
  return tid;
}

CustomCommsChannel::CustomCommsChannel(uint32_t id) {
  _rosChannelId = id;
  SetLogLevel(debug);
}

void CustomCommsChannel::SetMinPrTime(double prTime) {
  _minPrTime = prTime * 1000000;
} // from millis to nanos

void CustomCommsChannel::SetPrTimeInc(double inc) {
  _prTimeIncPerMeter = inc * 1000000;
} // from millis to nanos

void CustomCommsChannel::AddDevice(CustomROSCommsDeviceNs3Ptr dev) {
  _devices.push_back(dev);
}

void CustomCommsChannel::SendPacket(CustomROSCommsDeviceNs3Ptr dev,
                                    ns3PacketPtr pkt) {
  Debug("CustomCommsChannel: SendPacket");
  auto txpos = dev->GetPosition();
  // auto minErrRate = dev->GetMinPktErrorRate();
  // auto errRateInc = dev->GetPktErrorRateInc();

  for (CustomROSCommsDeviceNs3Ptr dst : _devices) {
    if (dst != dev) {
      auto rxpos = dst->GetPosition();
      auto distance = txpos.distance(rxpos);
      auto maxdm = dev->GetMaxDistance();
      auto mindm = dev->GetMinDistance();
      if (distance <= maxdm && distance >= mindm) { // dst is in range
        auto delay = _minPrTime + _prTimeIncPerMeter * distance;
        auto totalTime = static_cast<uint64_t>(round(delay));
        // auto errRate = minErrRate + errRateInc * distance;
        NetsimHeader header;
        pkt->RemoveHeader(header);
        auto propagationError = dev->ErrOnPkt(distance, pkt);
        pkt->AddHeader(header);
        //        if (error) {
        //          auto pBuffer = pkt->GetPayloadBuffer();
        //          *pBuffer = ~*pBuffer;
        //        }
        Debug("CustomCommsChannel: distance({} m) ; totalTime({} secs)",
              distance, totalTime / 1e9);
        ns3::Simulator::ScheduleWithContext(
            dev->GetMac(), NanoSeconds(totalTime),
            &CustomROSCommsDevice::AddNewPacket, dst, pkt, propagationError);
      }
    }
  }
}
}
