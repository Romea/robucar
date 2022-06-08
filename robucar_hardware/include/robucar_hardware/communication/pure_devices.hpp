#ifndef _ROBUCAR_COMMUNICATION_PURE_DEVICES_H
#define _ROBUCAR_COMMUNICATION_PURE_DEVICES_H

#include <string>
#include <vector>
#include <stdio.h>

#include <boost/thread.hpp>

namespace robucar_communication {

using namespace std;

class DataFrame
{
public:
  virtual size_t toBuffer (uint8_t *) = 0;
  virtual ~DataFrame() = 0;
};



/* ************************************************************************************************************* */
/*              Differential Drive Device                                                                        */
/* ************************************************************************************************************* */

// sent by outbound notification
class DataDiffDriveState : DataFrame {

private:
    float targetLinearSpeed;
    float currentLinearSpeed;
    float targetAngularSpeed;
    float currentAngularSpeed;

public:
    DataDiffDriveState (const uint8_t* buf);
    DataDiffDriveState (float tLS, float cLS, float tAS, float cAS) :
            targetLinearSpeed(tLS), currentLinearSpeed(cLS),
            targetAngularSpeed(tAS), currentAngularSpeed(cAS)
    {}

    float getTargetLinearSpeed() { return targetLinearSpeed; }
    float getCurrentLinearSpeed() { return currentLinearSpeed; }
    float getTargetAngularSpeed() { return targetAngularSpeed; }
    float getCurrentAngularSpeed() { return currentAngularSpeed; }
    size_t toBuffer (uint8_t* buf);
    void print();
};

// to be send with inbound notification
class DataDiffDriveCommand : DataFrame {
public:
    DataDiffDriveCommand(float tLS, float tAS) :
            targetLinearSpeed(tLS), targetAngularSpeed(tAS)
    {}
    DataDiffDriveCommand(const uint8_t *buf);
    float getTargetLinearSpeed() { return targetLinearSpeed; }
    float getTargetAngularSpeed() { return targetAngularSpeed; }
    size_t toBuffer (uint8_t* buf);
private:
    float targetLinearSpeed;
    float targetAngularSpeed;
};


// to be sent upon a GET request
class DataDiffDriveServiceState : DataFrame {

public:

    DataDiffDriveServiceState (const uint8_t* buf);

    DataDiffDriveServiceState(const float tls, const float cls, const float tas, const float cas) :
            targetLinearSpeed(tls), targetAngularSpeed(tas),
            currentLinearSpeed(cls), currentAngularSpeed(cas)
    {}

    float getTargetLinearSpeed() { return targetLinearSpeed; }
    float getTargetAngularSpeed() { return targetAngularSpeed; }
    float getCurrentLinearSpeed() { return currentLinearSpeed; }
    float getCurrentAngularSpeed() { return currentAngularSpeed; }

    float getMinLinearSpeed() { return minLinearSpeed; }
    float getMaxLinearSpeed() { return maxLinearSpeed; }
    float getMinAngularSpeed() { return minAngularSpeed; }
    float getMaxAngularSpeed() { return maxAngularSpeed; }

    float getMinLinearAcceleration() { return minLinearAcceleration; }
    float getMaxLinearAcceleration() { return maxLinearAcceleration; }
    float getMinAngularAcceleration() { return minAngularAcceleration; }
    float getMaxAngularAcceleration() { return maxAngularAcceleration; }


    void setTargetLinearSpeed(float value) { targetLinearSpeed = value; }
    void setTargetAngularSpeed(float value) { targetAngularSpeed = value; }
    void setCurrentLinearSpeed(float value) { currentLinearSpeed = value; }
    void setCurrentAngularSpeed(float value) { currentAngularSpeed = value; }

    void setMinLinearSpeed(float value) { minLinearSpeed = value; }
    void setMaxLinearSpeed(float value) { maxLinearSpeed = value; }
    void setMinAngularSpeed(float value) { minAngularSpeed = value; }
    void setMaxAngularSpeed(float value) { maxAngularSpeed = value; }

    void setMinLinearAcceleration(float value) { minLinearAcceleration = value; }
    void setMaxLinearAcceleration(float value) { maxLinearAcceleration = value; }
    void setMinAngularAcceleration(float value) { minAngularAcceleration = value; }
    void setMaxAngularAcceleration(float value) { maxAngularAcceleration = value; }

    float getWidth() { return width; }
    size_t toBuffer (uint8_t* buf);
    void print();

private:
    float targetLinearSpeed;
    float targetAngularSpeed;
    float currentLinearSpeed;
    float currentAngularSpeed;

    float maxLinearSpeed;
    float minLinearSpeed;
    float maxAngularSpeed;
    float minAngularSpeed;
    float maxLinearAcceleration;
    float minLinearAcceleration;
    float maxAngularAcceleration;
    float minAngularAcceleration;
    float width;

};

/* ************************************************************************************************************* */
/*               Drive Device                                                                                    */
/* ************************************************************************************************************* */

const static int DRIVE_LINEAR        = 0;
const static int DRIVE_ANGULAR       = 1;
const static int DRIVE_MODE_POSITION = 0;
const static int DRIVE_MODE_SPEED    = 1;
const static int DRIVE_MODE_TORQUE   = 2;
const static int DRIVE_ENABLE        = 1;
const static int DRIVE_DISABLE       = 0;


// used inside DataDriveServiceState
class DataDriveProperties
{
private:
    uint8_t driveType; // 0=linear 1=angular
    uint8_t defaultMode; // 0=position 1=velocity 2=torque
    float maximumPosition;
    float minimumPosition;
    float maximumSpeed;
    float minimumSpeed;
    float maximumAcceleration;
    float maximumTorque;
    float minimumTorque;

public:
    DataDriveProperties (const uint8_t*);
    DataDriveProperties (const uint8_t driveType, const uint8_t defaultMode,
                         float maximumPosition, float minimumPosition,
                         float maximumSpeed, float minimumSpeed,
                         float maximumAcceleration, float maximumTorque, float minimumTorque) :
    driveType(driveType), defaultMode(defaultMode), maximumPosition(maximumPosition),
    minimumPosition(minimumPosition), maximumSpeed(maximumSpeed),
    minimumSpeed(minimumSpeed), maximumAcceleration(maximumAcceleration),
    maximumTorque(maximumTorque), minimumTorque(minimumTorque)
    {}

    size_t toBuffer (uint8_t * buf);
    const static size_t LENGTH=30;
    void print();
};

// to be sent inside a notification
class DataDriveState {

private:
    uint8_t mode; // 0=position 1=velocity 2=torque
    uint8_t status; // 0=enabled 1=disabled 2=error
    float currentCommandValue;
    float currentPosition, currentSpeed, currentTorque;

public:
    const static size_t LENGTH=18;
    DataDriveState (const uint8_t*);
    DataDriveState (const uint8_t mode, const uint8_t status, const float commandedValue,
                    const float currentPosition, const float currentSpeed, const float currentTorque):
    mode(mode), status(status), currentCommandValue(commandedValue),
    currentPosition(currentPosition), currentSpeed(currentSpeed),
    currentTorque(currentTorque)
    {}
    size_t toBuffer (uint8_t* buf);
    uint8_t getMode() { return mode;}
    uint8_t getStatus() { return status;}
    float getCommandValue() { return currentCommandValue;}
    float getPosition () {return currentPosition;}
    float getSpeed () {return currentSpeed;}
    float getTorque() { return currentTorque;}
};

// sent by client to command motor
class DataDriveCommand {

public:
    DataDriveCommand (const uint8_t*);
    DataDriveCommand (const uint8_t enable, const uint8_t mode, const float command);
    static const size_t LENGTH=6;
    size_t toBuffer (uint8_t*);

private:
    uint8_t enable; // 0=enable 1=disable
    uint8_t mode; // 0=position 1=velocity 2=torque
    float commandValue;

};

// to be sent upon a GET request
class DataDriveServiceState : DataFrame {

private:
    uint32_t numberOfDrives;
    vector<boost::shared_ptr<DataDriveProperties> > drives;

public:
    DataDriveServiceState (const uint8_t* buf, const size_t dataLength);
    DataDriveServiceState (const unsigned int nbOfDrives) :
            numberOfDrives(nbOfDrives), drives()
    {}
    uint32_t noOfDrives () { return numberOfDrives; }
    size_t toBuffer (uint8_t* buf);
    void addDrive (boost::shared_ptr<DataDriveProperties> ddp) { drives.push_back(ddp); }
    boost::shared_ptr<DataDriveProperties> get (const unsigned int index);
    void print();

};


/* ************************************************************************************************************* */
/*              Car Drive Device                                                                                 */
/* ************************************************************************************************************* */

// to be send with inbound notification
class DataCarDriveCommand
{
public:
    DataCarDriveCommand(const uint8_t enable, const float linSpeed, const float steeringAngle);
    size_t toBuffer(uint8_t* buf);

private:
    const uint8_t enable;
    const float linSpeed, steeringAngle;

};

// sent by outbound notification
class DataCarDrive
{
public:
    DataCarDrive(const uint8_t* buf);
    DataCarDrive(float tSA, float CSA);
    void fromBuffer (const uint8_t* buf);
    float getTargetSteeringAngle() { return targetSteeringAngle; }
    float getCurrentSteeringAngle() { return currentSteeringAngle; }
private:
    float targetSteeringAngle, currentSteeringAngle;
};


/* ************************************************************************************************************* */
/*              Telemeters Device                                                                                */
/* ************************************************************************************************************* */

class DataTelemeters {
public:
    DataTelemeters(const uint8_t *buffer, const size_t);
    size_t toBuffer(uint8_t *buffer);
    vector<float> getDistances() const { return distances; }
private:
    vector<float> distances;
};

//
class DataTelemeterProperties {

public:
    DataTelemeterProperties (const uint8_t* buf);
    static const size_t LENGTH = 28;
    void print();
private:
    float xpos, ypos, orientation; // meters (???)
    float fieldOfView; // radians
    float minRange, maxRange;
    float currentMeasurement; // meters (what ever this might be)
};


// sent upon a GET request, containing a "DataTelemeterProperties" for each range finder device
class DataTelemetersServiceState : DataFrame {

public:
    DataTelemetersServiceState (const uint8_t* buf, const size_t dataLength);
    void print();
private:
    uint32_t numberOfDevices;
    vector<boost::shared_ptr<DataTelemeterProperties> > devices;

};


/* ************************************************************************************************************* */
/*              Laser Device                                                                                     */
/* ************************************************************************************************************* */

struct LaserEcho {
    size_t n;
    float angle, distance;
    LaserEcho(const uint8_t *buf) {
        n = sizeof(float);
        memcpy(&angle,    buf,   n);
        memcpy(&distance, buf+n, n);
    }
};

class DataLaser {
public:
    DataLaser(const uint8_t *buffer, const size_t);
    size_t toBuffer(uint8_t *buffer);
    vector<boost::shared_ptr<LaserEcho> > getDistances() const { return distances; }
    void print(ostream *str);
private:
    vector<boost::shared_ptr<LaserEcho> > distances;
};

//
class DataLaserProperties {

public:
    DataLaserProperties (const uint8_t* buf);
    static const size_t LENGTH = 28;
    void print();
private:
    float xpos, ypos, orientation;
    int32_t	numberOfMeasures;
};


// sent upon a GET request, containing a "DataLaserProperties" for each range finder device
class DataLaserServiceState {

public:
    DataLaserServiceState (const uint8_t* buf, const size_t dataLength);
    void print();
private:
    uint32_t numberOfDevices;
    boost::shared_ptr<DataLaserProperties> laserDevice;
};


/* ************************************************************************************************************* */
/*              Localization Device                                                                              */
/* ************************************************************************************************************* */

class DataLocalizationServiceState {

public:
    DataLocalizationServiceState (const uint8_t* buf);
    void print();
    double getXpos() { return xpos; }
    double getYpos() { return ypos; }
    double getOrientation() { return orientation; }


private:
    double xpos, ypos, orientation;
    uint8_t status;   // bit0 : accurancy is meter, bit1: accurancy is decimetric, bit2: accurancy is centimetric
    // bit3 : compute using proprioceptive sensors
    // bit4 : compute using exteroceptive sensors
};

} //end namespace robucar_communication

#endif // PURE_DEVICES_H
