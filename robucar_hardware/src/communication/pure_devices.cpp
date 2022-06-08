#include "string.h"
#include "robucar_hardware/communication/pure_devices.hpp"
#include <iostream>

namespace robucar_communication {

DataFrame::~DataFrame(){}

/* ************************************************************************************************************* */
/*              Differential Drive Device                                                                        */
/* ************************************************************************************************************* */

DataDiffDriveState::DataDiffDriveState(const uint8_t *buf)
{
    size_t len = 0, n = sizeof(float);
    memcpy(&targetLinearSpeed,   buf+len,    n);    len += n;
    memcpy(&currentLinearSpeed,  buf+len,    n);    len += n;
    memcpy(&targetAngularSpeed,  buf+len,    n);    len += n;
    memcpy(&currentAngularSpeed, buf+len,    n);    len += n;
}

size_t DataDiffDriveState::toBuffer (uint8_t* buf)
{
    memcpy(buf,    &targetLinearSpeed,   4);
    memcpy(buf+4,  &currentLinearSpeed,  4);
    memcpy(buf+8,  &targetAngularSpeed,  4);
    memcpy(buf+12, &currentAngularSpeed, 4);
    return 16;
}

void DataDiffDriveState::print()
{
    cout << "=============================================" << endl;
    cout << "Differential Drive state" << endl;
    cout << "=============================================" << endl;
    cout << "targetLinearSpeed      = " << targetLinearSpeed << endl;
    cout << "currentLinearSpeed     = " << currentLinearSpeed << endl;
    cout << "targetAngularSpeed     = " << targetAngularSpeed << endl;
    cout << "currentAngularSpeed    = " << currentAngularSpeed << endl;
    cout << "=============================================" << endl;
}

DataDiffDriveCommand::DataDiffDriveCommand(const uint8_t *buf)
{
    size_t len = 0, n = sizeof(float);
    memcpy(&targetLinearSpeed,   buf+len,    n);    len += n;
    memcpy(&targetAngularSpeed,  buf+len,    n);    len += n;
}

size_t DataDiffDriveCommand::toBuffer (uint8_t* buf)
{
    memcpy(buf,    &targetLinearSpeed,   4);
    memcpy(buf+4,  &targetAngularSpeed,  4);
    return 8;
}

DataDiffDriveServiceState::DataDiffDriveServiceState(const uint8_t *buf) {
    size_t len = 0, n = sizeof(float);

    memcpy(&targetLinearSpeed,      buf+len,    n);     len += n;
    memcpy(&targetAngularSpeed,     buf+len,    n);     len += n;
    memcpy(&currentLinearSpeed,     buf+len,    n);     len += n;
    memcpy(&currentAngularSpeed,    buf+len,    n);     len += n;

    memcpy(&maxLinearSpeed,         buf+len,    n);     len += n;
    memcpy(&minLinearSpeed,         buf+len,    n);     len += n;
    memcpy(&maxAngularSpeed,        buf+len,    n);     len += n;
    memcpy(&minAngularSpeed,        buf+len,    n);     len += n;
    memcpy(&maxLinearAcceleration,  buf+len,    n);     len += n;
    memcpy(&minLinearAcceleration,  buf+len,    n);     len += n;
    memcpy(&maxAngularAcceleration, buf+len,    n);     len += n;
    memcpy(&minAngularAcceleration, buf+len,    n);     len += n;
    memcpy(&width,                  buf+len,    n);     len += n;
}

void DataDiffDriveServiceState::print()
{
    cout << "=============================================" << endl;
    cout << "Differential Drive service state" << endl;
    cout << "=============================================" << endl;
    cout << "targetLinearSpeed      = " << targetLinearSpeed << endl;
    cout << "targetAngularSpeed     = " << targetAngularSpeed << endl;
    cout << "currentLinearSpeed     = " << currentLinearSpeed << endl;
    cout << "currentAngularSpeed    = " << currentAngularSpeed << endl;
    cout << "maxLinearSpeed         = " << maxLinearSpeed << endl;
    cout << "minLinearSpeed         = " << minLinearSpeed << endl;
    cout << "maxAngularSpeed        = " << maxAngularSpeed << endl;
    cout << "minAngularSpeed        = " << minAngularSpeed << endl;
    cout << "maxLinearAcceleration  = " << maxLinearAcceleration << endl;
    cout << "minLinearAcceleration  = " << minLinearAcceleration << endl;
    cout << "maxAngularAcceleration = " << maxAngularAcceleration << endl;
    cout << "minAngularAcceleration = " << minAngularAcceleration << endl;
    cout << "width                  = " << width << endl;
    cout << "=============================================" << endl;
}

size_t DataDiffDriveServiceState::toBuffer (uint8_t* buf)
{
    size_t len = 0, n = sizeof(float);
    memcpy(buf+len,    &targetLinearSpeed,   n);    len += n;
    memcpy(buf+len,    &targetAngularSpeed,   n);    len += n;
    memcpy(buf+len,    &currentLinearSpeed,   n);    len += n;
    memcpy(buf+len,    &currentAngularSpeed,   n);    len += n;
    memcpy(buf+len,    &maxLinearSpeed,   n);    len += n;
    memcpy(buf+len,    &minLinearSpeed,   n);    len += n;
    memcpy(buf+len,    &maxAngularSpeed,   n);    len += n;
    memcpy(buf+len,    &minAngularSpeed,   n);    len += n;
    memcpy(buf+len,    &maxLinearAcceleration,   n);    len += n;
    memcpy(buf+len,    &minLinearAcceleration,   n);    len += n;
    memcpy(buf+len,    &maxAngularAcceleration,   n);    len += n;
    memcpy(buf+len,    &minAngularAcceleration,   n);    len += n;
    memcpy(buf+len,    &width,   n);    len += n;
    return len;
}

/* ************************************************************************************************************* */
/*               Drive Device                                                                                    */
/* ************************************************************************************************************* */

//-------------------------------------
// Drive State
//-------------------------------------
DataDriveState::DataDriveState (const uint8_t* buffer)
{
    size_t len=0;
    memcpy(&mode, buffer, 1); len+=1;
    memcpy(&status, buffer+len, 1); len+=1;
    memcpy(&currentCommandValue, buffer+len, 4); len+=4;
    memcpy(&currentPosition, buffer+len, 4); len+=4;
    memcpy(&currentSpeed, buffer+len, 4); len+=4;
    memcpy(&currentTorque, buffer+len, 4);
}

size_t DataDriveState::toBuffer (uint8_t* buffer)
{
    size_t len=0;
    memcpy(buffer, &mode, 1); len+=1;
    memcpy(buffer+len, &status, 1); len+=1;
    memcpy(buffer+len, &currentCommandValue, 4); len+=4;
    memcpy(buffer+len, &currentPosition, 4); len+=4;
    memcpy(buffer+len, &currentSpeed, 4); len+=4;
    memcpy(buffer+len, &currentTorque, 4); len+=4;
    return len;
}


//-------------------------------------
// Drive Service State
//-------------------------------------
DataDriveServiceState::DataDriveServiceState (const uint8_t* buf, const size_t /*dataLength*/) :
        drives()
{
    size_t len=0;
    memcpy(&numberOfDrives, buf, 4); len+=4;

    for (size_t i=0; i<numberOfDrives; i++)
    {
        // create entries from buffer
        //printf("Drive %u\n", i);
        drives.push_back(boost::shared_ptr<DataDriveProperties>(new DataDriveProperties(buf+len)));
        len += DataDriveProperties::LENGTH;
    }
}

size_t DataDriveServiceState::toBuffer (uint8_t* buf)
{
    size_t len=0;

    memcpy(buf, &numberOfDrives, 4); len+=4;

    for (uint32_t i=0; i<numberOfDrives; i++) {
        //printf("Drive %u\n", i);
        len += drives.at(i)->toBuffer(buf+len);
    }
    return len;
}

boost::shared_ptr<DataDriveProperties> DataDriveServiceState::get(unsigned int index)
{
    return drives.at(index);
}

void DataDriveServiceState::print()
{
    cout << "=============================================" << endl;
    cout << "Drive service state" << endl;
    cout << "=============================================" << endl;
    vector<boost::shared_ptr<DataDriveProperties> >::iterator it;
    for (it=drives.begin(); it<drives.end(); it++)
    {
        (*it)->print();
    }
    cout << "=============================================" << endl;
}

//-------------------------------------
// Drive Command
//-------------------------------------
DataDriveCommand::DataDriveCommand(const uint8_t */*buf*/)
{
    // TODO copy data
}

DataDriveCommand::DataDriveCommand
        (const uint8_t enable, const uint8_t mode, const float command) :
        enable(enable), mode(mode), commandValue(command)
{}

size_t DataDriveCommand::toBuffer(uint8_t *buf)
{
    size_t len=0;
    memcpy(buf, &enable, 1); len+=1;
    memcpy(buf+len, &mode, 1); len+=1;
    memcpy(buf+len, &commandValue, 4); len+=4;
    return len;
}

//-------------------------------------
// Drive Properties
//-------------------------------------
size_t DataDriveProperties::toBuffer(uint8_t *buf)
{
    size_t len=0;

    memcpy(buf, &driveType, 1); len+=1;
    memcpy(buf+len, &defaultMode, 1); len+=1;
    memcpy(buf+len, &maximumPosition, 4); len+=4;
    memcpy(buf+len, &minimumPosition, 4); len+=4;
    memcpy(buf+len, &maximumSpeed, 4); len+=4;
    memcpy(buf+len, &minimumSpeed, 4); len+=4;
    memcpy(buf+len, &maximumAcceleration, 4); len+=4;
    memcpy(buf+len, &maximumTorque, 4); len+=4;
    memcpy(buf+len, &minimumTorque, 4); len+=4;

    return len;
}

// creates properties object from buffer
DataDriveProperties::DataDriveProperties(const uint8_t *buf) {
    size_t len=0;
    memcpy(&driveType, buf, 1); len+=1;
    memcpy(&defaultMode, buf+len, 1); len+=1;
    memcpy(&maximumPosition, buf+len, 4); len+=4;
    memcpy(&minimumPosition, buf+len, 4); len+=4;
    memcpy(&maximumSpeed, buf+len, 4); len+=4;
    memcpy(&minimumSpeed, buf+len, 4); len+=4;
    memcpy(&maximumAcceleration, buf+len, 4); len+=4;
    memcpy(&maximumTorque, buf+len, 4); len+=4;
    memcpy(&minimumTorque, buf+len, 4); len+=4;
}

void DataDriveProperties::print()
{
    printf("%u ", driveType);
    printf("%u ", defaultMode);
    cout << maximumPosition << " " << minimumPosition
            << " " << maximumSpeed << " " << minimumSpeed << " " << maximumAcceleration << " " << maximumTorque
            << " " << minimumTorque << endl;

}

/* ************************************************************************************************************* */
/*              Car Drive Device                                                                                 */
/* ************************************************************************************************************* */

//-------------------------------------
// Car Drive Command
//-------------------------------------
DataCarDriveCommand::DataCarDriveCommand(const uint8_t enable, const float linSpeed, const float steeringAngle) :
        enable(enable), linSpeed(linSpeed), steeringAngle(steeringAngle)
{}

size_t DataCarDriveCommand::toBuffer(uint8_t *buf)
{
    size_t len=0;
    memcpy(buf, &enable, 1); len+=1;
    memcpy(buf+len, &linSpeed, 4); len+=4;
    memcpy(buf+len, &steeringAngle, 4); len+=4;
    return len;
}

//-------------------------------------
// Car Drive
//-------------------------------------
void DataCarDrive::fromBuffer (const uint8_t* /*buf*/)
{
    //TODO
}

DataCarDrive::DataCarDrive(const uint8_t* buf)
{
    memcpy(&targetSteeringAngle,    buf,                sizeof(float));
    memcpy(&currentSteeringAngle,   buf+sizeof(float),  sizeof(float));
}

DataCarDrive::DataCarDrive(float tAS, float cAS)
{
    targetSteeringAngle = tAS;
    currentSteeringAngle = cAS;
}

/* ************************************************************************************************************* */
/*              Telemeters Device                                                                                */
/* ************************************************************************************************************* */
// REMARK : No PURE Service on PURE server
//-------------------------------------
// Temeters
//-------------------------------------
DataTelemeters::DataTelemeters(const uint8_t *buffer, const size_t dataLength) : distances()
{
    size_t N;
    if (!(dataLength%sizeof(float)))
    {
        N = dataLength/sizeof(float);
        distances.assign(buffer, buffer+N);
    }
}

//-------------------------------------
// Telemeters Service State
//-------------------------------------
DataTelemetersServiceState::DataTelemetersServiceState (const uint8_t* buf, const size_t /*dataLength*/) :
        devices()
{
    size_t len=0;
    memcpy(&numberOfDevices, buf, 4); len+=4;

    for (size_t i=0; i<numberOfDevices; i++)
    {
        // create entries from buffer
        //printf("Drive %u\n", i);
        devices.push_back(boost::shared_ptr<DataTelemeterProperties>(new DataTelemeterProperties(buf+len)));
        len += DataTelemeterProperties::LENGTH;
    }
}

void DataTelemetersServiceState::print()
{
    cout << "=============================================" << endl;
    cout << "Telemeter service state" << endl;
    cout << "(xpos ypos orientation fieldOfView minRange maxRange currentMeasurement)" << endl;
    cout << "=============================================" << endl;
    vector<boost::shared_ptr<DataTelemeterProperties> >::iterator it;
    for (it=devices.begin(); it<devices.end(); it++)
    {
        (*it)->print();
    }
    cout << "=============================================" << endl;
}

//-------------------------------------
// Telemeters Properties
//-------------------------------------
DataTelemeterProperties::DataTelemeterProperties (const uint8_t *buf)
{
    size_t len = 0, n=sizeof(float);
    memcpy(&xpos, buf+len, n); len+=n;
    memcpy(&ypos, buf+len, n); len+=n;
    memcpy(&orientation, buf+len, n); len+=n;
    memcpy(&fieldOfView, buf+len, n); len+=n;
    memcpy(&minRange, buf+len, n); len+=n;
    memcpy(&maxRange, buf+len, n); len+=n;
    memcpy(&currentMeasurement, buf+len, n); len+=n;
}

void DataTelemeterProperties::print()
{
    cout << xpos << " " << ypos
            << " " << orientation << " " << fieldOfView << " " << minRange << " " << maxRange
            << " " << currentMeasurement << endl;
}

/* ************************************************************************************************************* */
/*              Laser Device                                                                                     */
/* ************************************************************************************************************* */

//-------------------------------------
// Laser
//-------------------------------------
DataLaser::DataLaser(const uint8_t *buffer, const size_t dataLength) : distances()
{
    size_t N, n=2*sizeof(float);
    if (!(dataLength%n))
    {
        N = dataLength/n;
        for (size_t i=0; i<N; i++) {
            distances.push_back(boost::shared_ptr<LaserEcho>(new LaserEcho(buffer+i*n)));
        }
    }
}

void DataLaser::print(ostream *str)
{
    string result;
    vector<boost::shared_ptr<LaserEcho> >::iterator it;
    for (it=distances.begin(); it<distances.end(); it++) {
        *str << (*it)->angle << " " << (*it)->distance << " ";
    }
}

//-------------------------------------
// Laser Service State
//-------------------------------------
DataLaserServiceState::DataLaserServiceState (const uint8_t* buf, const size_t /*dataLength*/)
{
    laserDevice = boost::shared_ptr<DataLaserProperties>(new DataLaserProperties(buf));
}

void DataLaserServiceState::print()
{
    cout << "=============================================" << endl;
    cout << "Laser service state" << endl;
    cout << "(xpos ypos orientation numberOfMeasures)" << endl;
    cout << "=============================================" << endl;
    laserDevice->print();
    cout << "=============================================" << endl;
}

//-------------------------------------
// Laser Properties
//-------------------------------------
DataLaserProperties::DataLaserProperties (const uint8_t *buf)
{
    size_t len = 0, n=sizeof(float);
    memcpy(&xpos, buf+len, n); len+=n;
    memcpy(&ypos, buf+len, n); len+=n;
    memcpy(&orientation, buf+len, n); len+=n;
    memcpy(&numberOfMeasures, buf+len, n); len+=n; // 4 bytes as well, even if its "int32"
}

void DataLaserProperties::print()
{
    cout << xpos << " " << ypos
            << " " << orientation << " " << numberOfMeasures << endl;
}

/* ************************************************************************************************************* */
/*              Localization Device                                                                              */
/* ************************************************************************************************************* */

//-------------------------------------
// Localisation Service State
//-------------------------------------
DataLocalizationServiceState::DataLocalizationServiceState(const uint8_t *buf)
{
    size_t len = 0, n= sizeof(double);
    memcpy(&xpos, buf+len, n); len += n;
    memcpy(&ypos, buf+len, n); len += n;
    memcpy(&orientation, buf+len, n); len += n;
    memcpy(&status, buf+len, 1); len += 1;
}

void DataLocalizationServiceState::print()
{
    cout << "=============================================" << endl;
    cout << "Localization service state" << endl;
    cout << "(xpos ypos orientation status)" << endl;
    cout << "=============================================" << endl;
    cout << "'" << xpos << "' '" << ypos
            << "' '" << orientation << "' '" << endl;
    printf("%X\n", status);
    cout << "=============================================" << endl;
}

} //end namespace robucar_communication
