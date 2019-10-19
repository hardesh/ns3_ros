#include <dccomms/Packet.h>
#include <iostream>
#include <ns3/header.h>
#include <ns3/packet.h>
#include <ns3/ptr.h>

namespace dccomms_ros {

using namespace ns3;

class NetsimHeader : public Header {
public:
  NetsimHeader();
  virtual ~NetsimHeader();

  void SetSeqNum(uint64_t data);
  void SetPacketSize(uint32_t size);
  void SetDst(uint32_t addr);
  void SetSrc(uint32_t addr);
  void SetPacketError(bool v);
  void SetNanosPerByte(uint64_t npb);

  uint64_t GetSeqNum(void) const;
  uint64_t GetNanosPerByte(void) const;
  uint32_t GetPacketSize(void) const;
  uint32_t GetDst(void) const;
  uint32_t GetSrc(void) const;
  bool GetPacketError() const;

  static TypeId GetTypeId(void);
  virtual TypeId GetInstanceTypeId(void) const;
  virtual void Print(std::ostream &os) const;
  virtual void Serialize(Buffer::Iterator start) const;
  virtual uint32_t Deserialize(Buffer::Iterator start);
  virtual uint32_t GetSerializedSize(void) const;

  static NetsimHeader Build(dccomms::PacketPtr pkt) {
    NetsimHeader header;
    header.SetSeqNum(0);
    header.SetNanosPerByte(0);
    header.SetDst(pkt->GetDestAddr());
    header.SetSrc(pkt->GetSrcAddr());
    header.SetPacketSize(pkt->GetPacketSize());
    header.SetPacketError(pkt->PacketIsOk());
    return header;
  }

private:
  uint64_t _seq, _npb;
  uint32_t _packetSize, _dst, _src;
  bool _error;
};
}
