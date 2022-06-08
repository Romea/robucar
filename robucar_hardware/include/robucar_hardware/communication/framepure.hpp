#ifndef _ROBUFAST_COMMUNICATION_PURE_DRIVE_FRAME_HPP_
#define _ROBUFAST_COMMUNICATION_PURE_DRIVE_FRAME_HPP_

#include "robucar_hardware/communication/pure_interface.hpp"

#include <vector>
#include <boost/shared_ptr.hpp>

namespace robucar_communication {

class FramePureDrive
{
public :

  FramePureDrive(const uint8_t * inBuffer, unsigned int size);

  void decode();

  boost::shared_ptr<DataDriveState> getMotorState (size_t index);

  static const unsigned int FL=0, FR=1, RL=3, RR=4, FS=2, RS=5;

private:
  const static size_t MAX_SIZE = 4096;
  bool isDecoded_;
  unsigned int size_;
  uint8_t buffer_[MAX_SIZE];

  std::vector<boost::shared_ptr<DataDriveState> > motorData;
};


class CommandPureDrive
{
public:
  size_t toBuffer(uint8_t*);
  // parameter: PURE target id, enable
  CommandPureDrive(const uint8_t enable=DRIVE_DISABLE);
  void setSpeedFR(const float speedFR);
  void setSpeedFL(const float speedFL);
  void setSpeedRR(const float speedRR);
  void setSpeedRL(const float speedRL);
  void setFrontSteering(const float frontSteering);
  void setRearSteering(const float rearSteering);
  void setDriveStatus(const uint8_t drive_status);
private:
  float speedFR_, speedFL_, speedRR_, speedRL_, frontSteering_, rearSteering_;
  uint8_t enable;
};

} //end namespace robufast_communication

#endif
