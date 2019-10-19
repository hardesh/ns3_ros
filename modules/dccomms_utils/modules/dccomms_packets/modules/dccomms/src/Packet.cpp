#include <cstring>
#include <dccomms/Packet.h>
#include <stdint.h>
#include <stdlib.h>

namespace dccomms {

Packet::Packet() {
  _buffer = NULL;
  _ownBuffer = true;
  uint32_t word = 0x1;
  uint8_t *byte = (uint8_t *)&word;
  _bigEndian =  *byte != 0x1;
}

Packet::~Packet() { _FreeBuffer(); }

void Packet::_FreeBuffer() {
  if (_ownBuffer && _buffer) {
    delete _buffer;
    _buffer = NULL;
  }
}

void Packet::_AllocBuffer(int size) {
  _FreeBuffer();
  _buffer = new uint8_t[size];
  _ownBuffer = true;
}

uint32_t Packet::SetPayload(uint8_t *data) {
  uint32_t psize = GetPayloadSize();
  uint8_t * pb = GetPayloadBuffer();
  memcpy(pb, data, psize);
  PayloadUpdated(psize);
  return psize;
}

void Packet::Write(Stream *comms) {
  comms->Write(GetBuffer(), GetPacketSize());
}

void Packet::_SetBuffer(void *buffer) {
  _FreeBuffer();
  _buffer = (uint8_t *)buffer;
  _ownBuffer = false;
}

bool Packet::PacketIsOk() { return true; }
}
