#include <dccomms_packets/SimplePacket.h>
#include <class_loader/multi_library_class_loader.h>

namespace dccomms_packets {
SimplePacket::SimplePacket(int payloadSize, FCS fcs) {
  FCS_SIZE = fcs == CRC16 ? 2 : 0;
  PAYLOAD_SIZE = payloadSize;
  _packetSize = PRE_SIZE + PAYLOAD_SIZE + FCS_SIZE;
  _AllocBuffer(_packetSize);
  _Init();
}

void SimplePacket::_Init() {
  _pre = GetBuffer();
  *_pre = 0x55;
  _payload = _pre + PRE_SIZE;
  _fcs = _payload + PAYLOAD_SIZE;
}

void SimplePacket::CopyFromRawBuffer(void *buffer) {
  memcpy(GetBuffer(), buffer, _packetSize);
}

inline uint8_t *SimplePacket::GetPayloadBuffer() { return _payload; }

inline uint32_t SimplePacket::GetPayloadSize() { return PAYLOAD_SIZE; }

inline int SimplePacket::GetPacketSize() { return _packetSize; }

void SimplePacket::Read(Stream *stream) {
  stream->WaitFor(_pre, PRE_SIZE);
  stream->Read(_payload, PAYLOAD_SIZE + FCS_SIZE);
}

void SimplePacket::PayloadUpdated(uint32_t payloadSize) { UpdateFCS(); }

void SimplePacket::GetPayload(void *copy, int size) {
  auto copySize = PAYLOAD_SIZE < size ? PAYLOAD_SIZE : size;
  memcpy(copy, _payload, copySize);
}

uint32_t SimplePacket::SetPayload(uint8_t *data, uint32_t size) {
  auto copySize = PAYLOAD_SIZE < size ? PAYLOAD_SIZE : size;
  memcpy(_payload, data, copySize);
  return copySize;
}

void SimplePacket::UpdateFCS() {
  switch (FCS_SIZE) {
  case 2: {
    uint16_t crc = Checksum::crc16(_payload, PAYLOAD_SIZE);
    *_fcs = (uint8_t)(crc >> 8);
    *(_fcs + 1) = (uint8_t)(crc & 0xff);
  } break;
  default:
    break;
  }
}

bool SimplePacket::_CheckFCS() {
  switch (FCS_SIZE) {
  case 2: {
    uint16_t crc = Checksum::crc16(_payload, PAYLOAD_SIZE + FCS_SIZE);
    return crc == 0;
  }
  default:
    return true;
  }
}
bool SimplePacket::PacketIsOk() { return _CheckFCS(); }

CLASS_LOADER_REGISTER_CLASS(SimplePacketBuilder20crc16, IPacketBuilder)
CLASS_LOADER_REGISTER_CLASS(SimplePacketBuilder9crc16, IPacketBuilder)
CLASS_LOADER_REGISTER_CLASS(SimplePacketBuilder109crc16, IPacketBuilder)
}


