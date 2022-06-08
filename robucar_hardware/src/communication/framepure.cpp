#include "robucar_hardware/communication/framepure.hpp"

namespace robucar_communication {

FramePureDrive::FramePureDrive(const uint8_t * buffer, unsigned int size):
  isDecoded_(false), size_(size), buffer_()
{
  memcpy(buffer_, buffer, size);
}

void FramePureDrive::decode()
{
  if (!isDecoded_) {

    // determine size provisional (maybe not that provisional, I don't see a
    // possibility to get the number of drives here)
    if (!(size_ % DataDriveState::LENGTH))
    {
      size_t nrOfDrives = size_ / DataDriveState::LENGTH;

      for (size_t i=0; i<nrOfDrives; i++) {
        motorData.push_back
            (boost::shared_ptr<DataDriveState>
             (new DataDriveState(buffer_ + i*DataDriveState::LENGTH))
             );
      }

    }
  }
  isDecoded_ = true;
}

boost::shared_ptr<DataDriveState> FramePureDrive::getMotorState (size_t index)
{
  return motorData.at(index);
}


/* ************************************************************************** */

CommandPureDrive::CommandPureDrive(const uint8_t enable) :
    speedFR_(0), speedFL_(0), speedRR_(0), speedRL_(0), frontSteering_(0),
    rearSteering_(0), enable(enable)
{}

void CommandPureDrive::setSpeedFR(const float speedFR) { speedFR_ = speedFR; }
void CommandPureDrive::setSpeedFL(const float speedFL) { speedFL_ = speedFL; }
void CommandPureDrive::setSpeedRR(const float speedRR) { speedRR_ = speedRR; }
void CommandPureDrive::setSpeedRL(const float speedRL) { speedRL_ = speedRL; }
void CommandPureDrive::setFrontSteering(const float frontSteering) { frontSteering_ = frontSteering; }
void CommandPureDrive::setRearSteering(const float rearSteering) { rearSteering_ = rearSteering; }
void CommandPureDrive::setDriveStatus(const uint8_t drive_status) { enable = drive_status; }

size_t CommandPureDrive::toBuffer(uint8_t *buf)
{
  // prepare structures

    // --------------------------------------------------------------------------------------------------------------
    // UPDATE : Une modif ici cause MAJ 16/05/2011
    // --------------------------------------------------------------------------------------------------------------
  /*DataDriveCommand commands[6] = {
    DataDriveCommand (enable, DRIVE_MODE_SPEED, speedFL_),
    DataDriveCommand (enable, DRIVE_MODE_SPEED, speedFR_),
    DataDriveCommand (enable, DRIVE_MODE_SPEED, speedRL_),
    DataDriveCommand (enable, DRIVE_MODE_SPEED, speedRR_),
    DataDriveCommand (enable, DRIVE_MODE_POSITION, frontSteering_),
    DataDriveCommand (enable, DRIVE_MODE_POSITION, rearSteering_)
  };*/
    DataDriveCommand commands[6] = {
        DataDriveCommand (enable, DRIVE_MODE_SPEED, speedFL_),
        DataDriveCommand (enable, DRIVE_MODE_SPEED, speedFR_),
        DataDriveCommand (enable, DRIVE_MODE_POSITION, frontSteering_),
        DataDriveCommand (enable, DRIVE_MODE_SPEED, speedRL_),
        DataDriveCommand (enable, DRIVE_MODE_SPEED, speedRR_),
        DataDriveCommand (enable, DRIVE_MODE_POSITION, rearSteering_)
      };

  // header
  //NotificationInboundHeader nih(target);

  size_t len=0;
  // copy data
  //len = nih.toBuffer(buf);
  for (size_t i=0; i<6; i++) {
    len += commands[i].toBuffer(buf+len);
  }
  return len;
}

} //end namespace robufast_communication
