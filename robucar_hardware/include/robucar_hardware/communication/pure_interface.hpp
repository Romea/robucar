#ifndef _ROBUCAR_COMMUNICATION_PURE_INTERFACE_H
#define _ROBUCAR_COMMUNICATION_PURE_INTERFACE_H

#include <queue>
#include <vector>
#include <string.h>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include "pure_devices.hpp"

namespace robucar_communication {

using namespace std;

const static size_t BUF_SIZE = 4096;

// function pointers to data handles
typedef void (*HandleFun)(uint8_t*, size_t);

struct OutboundNotification;
typedef boost::shared_ptr<OutboundNotification> OutboundNotificationPtr;

class Pure
{
  public:
  static const uint8_t ACTION_GET     = 0;
  static const uint8_t ACTION_QUERY   = 1;
  static const uint8_t ACTION_REPLACE = 2;
  static const uint8_t ACTION_UPDATE  = 3;
  static const uint8_t ACTION_INSERT  = 4;
  static const uint8_t ACTION_DELETE  = 5;
  static const uint8_t ACTION_SUBMIT  = 6;

  static const uint8_t ERROR_SUCCESS              = 0;
  static const uint8_t ERROR_NO_SUCH_TARGET       = 1;
  static const uint8_t ERROR_ACTION_NOT_SUPPORTED = 2;
  static const uint8_t ERROR_UNKNOWN_ACTION       = 3;
  static const uint8_t ERROR_INVALID_LENGTH       = 4;
  static const uint8_t ERROR_INVALID_DATA         = 5;

  // the two constant targets
  static const uint8_t TARGET_DIR           = 0; // directory service
  static const uint8_t TARGET_NOTIFICATION  = 1; // notification manager

  static const uint8_t ID_ERROR         = 0;
  static const uint8_t ID_NOTIFICATION  = 255;

  // generic types
  static const uint16_t TYPE_DIFF_DRIVE   = 0x4005;
  static const uint16_t TYPE_IO           = 0x4001;
  static const uint16_t TYPE_LASER        = 0x4004;
  static const uint16_t TYPE_TELEMETER    = 0x4008;
  static const uint16_t TYPE_DRIVE        = 0x4009;
  static const uint16_t TYPE_LOCALIZATION = 0x8002;

  // message header lengths
  static const size_t   LENGTH_RESPONSE          = 5;
  static const size_t   LENGTH_OUTB_NOTIFICATION = 11;

  static string errorMessage (uint8_t code) {
    switch (code)
    {
    case 0: return "no error";
    case 1: return "no such target";
    case 2: return "action not supported";
    case 3: return "unknown action";
    case 4: return "invalid length";
    case 5: return "invalid data";
    default: return "undefined error";
    }
  }

};


struct PureRequestHeader {

  uint8_t id, action;
  uint16_t target;

//  size_t toBuffer (Buffer buffer) {
  //  return toBuffer(buffer.)
  //}

  size_t toBuffer (uint8_t* data) {
    size_t len=0;
    data[len++] = id;
    data[len++] = action;
    memcpy(data+len, &target, 2);
    len += 2;
    return len;
  }

  size_t fromBuffer (uint8_t* data) {
    size_t len=0;
    id = data[len++];
    action = data[len++];
    memcpy(&target, data+len, 2);
    len += 2;
    return len;
  }

  PureRequestHeader() {}

  // see protocol description
  PureRequestHeader (uint8_t id, uint8_t action, uint16_t target):
      id(id), action(action), target(target) {}

};

struct PureResponseHeader {

  uint8_t id, action, error;
  uint16_t target;

  size_t toBuffer (uint8_t* data) {
    size_t len=0;
    data[len++] = id;
    data[len++] = action;
    memcpy(data+len, &target, 2);
    len += 2;
    data[len++] = error;
    return len;
  }

  size_t fromBuffer (const uint8_t* data) {
    size_t len=0;
    id = data[len++];
    action = data[len++];
    memcpy(&target, data+len, 2);
    len += 2;
    error = data[len++];
    return len;
  }

  PureResponseHeader() {}

  PureResponseHeader (const uint8_t* data) {
    size_t len=0;
    id = data[len++];
    action = data[len++];
    memcpy(&target, data+len, 2);
    len += 2;
    error = data[len++];
    printf("Created response header: %d/%d/%d\n", id, action, target);
  }

  // used to reply to a request in a simple way
  PureResponseHeader
      (const PureRequestHeader reqHeader, const uint8_t error = Pure::ERROR_SUCCESS) :
      id(reqHeader.id), action(reqHeader.action), error(error), target(reqHeader.target) {}

  // see protocol description
  PureResponseHeader
      (const uint8_t id, const uint8_t action, const uint16_t target, const uint8_t error) :
      id(id), action(action), error(error), target(target) {}

};

struct PureResponse {

  PureResponse() : header() {}

  PureResponse(const uint8_t* buf, size_t length)
    : header(buf), dataLength(length-Pure::LENGTH_RESPONSE)
  {
    memcpy(dataBuffer, buf+Pure::LENGTH_RESPONSE, dataLength);
#ifndef NDEBUG
    printf
      ("Created Response object with %d bytes of data (id=%u, action=%u, target=%u)\n",
      int(dataLength), header.id, header.action, header.target);
#endif
  }

  bool matches (uint8_t id, uint8_t action, uint16_t target)
  {
    return (header.id==id && header.action==action && header.target==target);
  }

  PureResponseHeader header;
  uint8_t dataBuffer[BUF_SIZE];
  size_t dataLength;
};

// an entry in the internal directory of the PURE controller
struct DirEntry {

  uint16_t type;
  uint16_t instance;
  std::string description;

  DirEntry(uint8_t* data) {
    memcpy(&type, data, 2);
    memcpy(&instance, data+2, 2);
  }
  DirEntry(uint16_t t, uint16_t i, string descr=""): type(t), instance(i), description(descr) {}

  void toBuffer (uint8_t* data) {
    //printf("converting to buffer: type=0x%02X instance=0x%02X\n", type, instance);
    memcpy(data, &type, 2);
    memcpy(data+2, &instance, 2);
  }

};

// used to store a received directory in an easy way; necessary to identify the different sensor types
struct Directory {

  // number of entries
  uint16_t len;
  // array of entires
  vector<DirEntry*> dir;

  // convert to byte array
  int toBuffer (uint8_t* data)
  {
    // send all entries
    for (size_t i=0; i<len; i++)
      dir[i]->toBuffer(data+i*4);
      //memcpy(data+2+i*sizeof(DirEntry), (void*)(&(dir[i])), 4);
    return (4*len);
  }

  // create from byte array (without the number of entries in the response message)
  void fromBuffer(uint8_t* data, int nb_entries)
  {
   len = nb_entries;
    for (int i=0; i<len; i++) {
      dir.push_back(new DirEntry(data+i*4));
    }
  }


  bool getInstanceDescription (const uint16_t instance, string& description) {
    for (vector<DirEntry*>::iterator it=dir.begin(); it<dir.end(); it++)
    {
      if ((*it)->instance == instance) {
        description = (*it)->description;
        return true;
      }
    }
    return false;
  }

  void setInstancsDescription (const uint16_t instance, const string description) {
    for (vector<DirEntry*>::iterator it=dir.begin(); it<dir.end(); it++)
    {
      if ((*it)->instance == instance) {
        (*it)->description = description;
        return;
      }
    }
  }

  bool getInstanceType (const uint16_t instance, uint16_t* type) {
    for (vector<DirEntry*>::iterator it=dir.begin(); it<dir.end(); it++)
    {
      if ((*it)->instance == instance) {
        *type = (*it)->type;
        return true;
      }
    }
    return false;
  }

  void print () {

    printf("================================================\n");
    printf("Directory has %d entries\n", len);
    printf("================================================\n");
    printf(" type      instance    description\n");
    printf("------------------------------------------------\n");
    for (vector<DirEntry*>::iterator it=dir.begin(); it<dir.end(); it++)
      printf(" 0x%04X    0x%04X      %s\n", (*it)->type, (*it)->instance, (*it)->description.c_str());
    printf("================================================\n");

  }

  ~Directory()
  {
    for (size_t i=0; i<len; i++)
      delete dir[i];
  }
};


struct NotificationStop {

  const uint16_t instance;
  NotificationStop(const uint16_t instance): instance(instance) {}

  size_t toBuffer (uint8_t* data) {
    size_t len=0;
    memcpy(data, &instance, 2);
    len += 2;
    return len;
  }
};

struct NotificationRequest {

  const uint16_t instance;
  const uint8_t period;

  NotificationRequest(const uint16_t instance, const uint8_t period) :
      instance(instance), period(period)
  {}

  size_t toBuffer (uint8_t* data) {
    size_t len=0;
    memcpy(data, &instance, 2);
    len += 2;
    memcpy(data+(len++), &period, 1);
    //printf("i %d p %d\n", instance, period);
    return len;
  }
};

class NotificationEntry {

public:
  const bool hasHandle;
  const HandleFun handle;
  const uint16_t instance;
private:
  unsigned int historySize;
  queue<OutboundNotificationPtr> history;
  boost::mutex accessMutex;//, dataReadyMutex;
  boost::timed_mutex dataReadyMutex;

public:
  NotificationEntry (const uint16_t instance, const HandleFun handle) :
      hasHandle(true), handle(handle), instance(instance)
  {
    printf("Creating notification entry with handle (target=%u)\n", instance);
  }

  NotificationEntry (const uint16_t instance, const unsigned int histSize) :
      hasHandle(false), handle(NULL), instance(instance),
      historySize(histSize), history()
  {
    printf("Creating notification entry without handle (target=%u)\n", instance);
    dataReadyMutex.lock();
  }

  //
  // add message to buffer
  //
  void enqueue (OutboundNotificationPtr newNotification)
  {
    accessMutex.lock();

    history.push(newNotification);

    if (history.size() > historySize)
      history.pop();
    // unlock wait mutex (preventively)
    dataReadyMutex.unlock();

    accessMutex.unlock();
  }

  //
  // pops the oldest message from the FIFO buffer, 3 different possibilities to control
  // the behaviour via "timeoutMs":
  //  1) timeoutMs < 0      call blocks, until data is available
  //  2) timeoutMs = 0      call is not blocking and returns NULL if no data is in buffer
  //  3) timeoutMs > 0      call is blocking until data is available or <timeoutMs> ms
  //                        are exceeded
  //
  // returns: a smart pointer on an OutboundNotifiction instance or NULL
  //
  OutboundNotificationPtr pop (const int timeoutMs)
  {

    OutboundNotificationPtr p;

    accessMutex.lock();
    if (history.size()>0) { // if message in buffer return it, mode not important
      p = history.front();
      history.pop();
      accessMutex.unlock();
      return p;
    }
    else                    // no message
    {
      // do not wait
      if (timeoutMs==0)
      {
        accessMutex.unlock();
        return p; // return NULL pointer
      }
      // wait until data is available
      else if (timeoutMs<0)
      {
        accessMutex.unlock(); // let the data be accessible for write
        dataReadyMutex.lock();
        // blocks until data is written

        // get latest element
        accessMutex.lock();
        p = history.front();
        history.pop();
        accessMutex.unlock();
        return p;
      }
      // wait until data is availabe or timeout occured
      else
      {
        accessMutex.unlock(); // let the data be accessible for write
        dataReadyMutex.timed_lock(boost::posix_time::milliseconds(timeoutMs));
        // blocks until data is written or timeout received

        accessMutex.lock();
        if (history.size()>0)
        {
          // get latest element
          p = history.front();
          history.pop();
        }
        accessMutex.unlock();
        return p;
      }
    }


//    return p;
  }
};

struct Notifications {
  vector<NotificationEntry*> entries;
};

struct NotificationOutboundHeader {

  const uint8_t id;
  uint16_t target;
  uint64_t timestamp;

  NotificationOutboundHeader() : id(255) {}

  // copy constructor
  NotificationOutboundHeader(const NotificationOutboundHeader& header) :
          id(255), target(header.target), timestamp(header.timestamp)
  {}

  // constrcut from parameters
  NotificationOutboundHeader (uint16_t target, uint64_t ts) :
          id(255), target(target), timestamp(ts)
  {}

  size_t fromBuffer (const uint8_t *data) {
    size_t len=1; // do not copy first byte, always 255
    memcpy(&target, data+len, 2);
    len += 2;
    memcpy(&timestamp, data+len, 8);
    len += 8;
    return len;
  }

  size_t toBuffer (uint8_t *data) {
    size_t len=0;
    memcpy(data+(len++), &id, 1);
    memcpy(data+len, &target, 2);
    len += 2;
    memcpy(data+len, &timestamp, 8);
    len += 8;
    return len;
  }
};

struct OutboundNotification {
  NotificationOutboundHeader header;
  uint8_t dataBuffer[BUF_SIZE];
  size_t dataLength;

  OutboundNotification
      (const NotificationOutboundHeader header, const uint8_t* data, const size_t length):
      header(header), dataLength(length)
  {
    memcpy(dataBuffer, data, length);
  }
};

class NotificationInboundHeader {

private:
  const static size_t LENGTH=3;
  const uint16_t target;
  const uint8_t id;// = 0xFF;

public:
  NotificationInboundHeader (const uint16_t target);
  size_t toBuffer(uint8_t* buf);

};

} //end namespace robucar_communication

#endif // PURE_INTERFACE_H
