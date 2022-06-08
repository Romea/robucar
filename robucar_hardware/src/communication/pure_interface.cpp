
#include "robucar_hardware/communication/pure_interface.hpp"

namespace robucar_communication {

NotificationInboundHeader::NotificationInboundHeader (const uint16_t target) :
    target(target), id(0xFF) {}


size_t NotificationInboundHeader::toBuffer(uint8_t* buf)
{
  size_t len=0;
  memcpy(buf, &id, 1); len+=1;
  memcpy(buf+len, &target, 2); len+=2;
  return len;
}

} //end namespace robucar_communication
