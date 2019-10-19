#ifndef DCCOMMS_PACKET_H
#define DCCOMMS_PACKET_H

#include <dccomms/Stream.h>
#include <iostream>
#include <memory>

namespace dccomms {

class Packet;
typedef std::shared_ptr<Packet> PacketPtr;

class Packet {
public:
  Packet();
  virtual ~Packet();
  inline uint8_t *GetBuffer() const { return _buffer; }

  virtual void CopyFromRawBuffer(void *buffer) = 0;
  virtual uint8_t *GetPayloadBuffer() = 0;
  virtual uint32_t GetPayloadSize() = 0;
  virtual int GetPacketSize() = 0;
  virtual void Read(Stream *comms) = 0;

  // This method could update the FCS or attributes of the subclass
  virtual void PayloadUpdated(uint32_t payloadSize = 0) = 0;

  // This method calls SetPayload(data, GetPayloadSize()) and
  // PayloadUpdated(GetPayloadSize())
  // Then returns the number of bytes written
  virtual uint32_t SetPayload(uint8_t *data);

  // It tries to set datasize bytes in the payload buffer and returns the number
  // of bytes written
  virtual uint32_t SetPayload(uint8_t *data, uint32_t datasize) = 0;

  virtual bool PacketIsOk();
  virtual void Write(Stream *comms);
  virtual bool IsBroadcast() { return true; }
  virtual uint32_t GetDestAddr() { return 0; }
  virtual uint32_t GetSrcAddr() { return 0; }
  virtual void SetDestAddr(uint32_t ddir){}
  virtual void SetSrcAddr(uint32_t sdir){}

protected:
  void _AllocBuffer(int size);
  virtual void _SetBuffer(void *buffer);
  bool _bigEndian;

private:
  uint8_t *_buffer;
  bool _ownBuffer;
  void _FreeBuffer();
};
}
#endif // DCCOMMS_PACKET_H
