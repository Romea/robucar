#ifndef _ROBUCAR_COMMUNICATION_PURE_CLIENT_H
#define _ROBUCAR_COMMUNICATION_PURE_CLIENT_H

#include <string>
#include <deque>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include "pure_interface.hpp"

namespace robucar_communication {

//#define BUF_SIZE 4096

using boost::asio::ip::udp;

/**********************************************************/

typedef boost::shared_ptr<PureResponse> ResponsePtr;

/**********************************************************/
class PureClient
{
/**********************/
public:

  // singleton functions
  static PureClient& getInstance (const std::string host, const unsigned int port);

  // returns true if socket could be created
  bool socketValid ();

  // requests the PURE directory
  void getDirectory ();

  // prints current directory to std out (including descriptions)
  void printDirectory ();

  // launch a reception thread, only allowed if callback functions have been registered for
  // all notification requests
  bool startThreadedReception ();

  // requests a notification for sensor <instance> in the directory, period is
  // an integer muliple of 40 ms by default, a callback function pointer is mandatory
  void requestNotification (const uint16_t instance, const uint8_t period, HandleFun handle);

  // requests a notification as before, but no callback function is used, thus a buffer
  // for <historySize> messages will be used -> get messages via "getMessage"
  // returns a pointer to the interface object or NULL in case of a communication error
  NotificationEntry* requestNotification
      (const uint16_t instance, const uint8_t period, const int historySize=1);

  // requests to end notifications for target "instance"
  int requestNotificationStop (const uint16_t instance);

  // sends a generic request to target "target" without additional data
  bool sendRequest (const uint8_t id, const uint8_t action, const uint16_t target);

  // read first message in FIFO buffer, if <timeout>!=0 is specified the method will block
  //size_t getMessage (const int16_t target, uint8_t* buf, const int timeoutMs=0);

  // returns the last administrative message recieved for the given parameters
  ResponsePtr findResponse
      (const uint8_t id, const uint8_t action,
       const uint16_t target, unsigned int waitMs=300);

  // send notification to server
  size_t sendNotification (const uint16_t target, const uint8_t *buf, const size_t len);

  // send data to server, returns number of bytes send, 0 on error
  size_t send (const uint8_t* buf, const size_t len);

/**********************/
private:

  // private constructor due to singleton
  // creates a UDP socket to <host:port>
  PureClient (const std::string host, const unsigned int port);
  PureClient (const PureClient&); // avoid compiler generation as public
  ~PureClient ();

  // Network
  boost::asio::io_service io_service;
  udp::resolver resolver;
  udp::resolver::query query;
  udp::endpoint receiver, sender;
  udp::socket socket;
  void startAsyncReception();
  void handle_receive_from(const boost::system::error_code& error, size_t bytes_recvd);

  // Pure
  Directory directory;
  Notifications notifications;
  void parseMessage (size_t len);
  bool sendNotificationRequest (const uint8_t action, const uint16_t instance,
                                const uint8_t period);

  // Threading
  boost::thread *thrd;
  void runReception();
  bool run;
  boost::mutex mutex;
  // receives and parses a message, return true if a message was received
  void receiveMessage ();

  // Buffers
  deque<ResponsePtr> responseMessages;
  uint8_t recvBuf[BUF_SIZE];


};

} //end namespace robucar_communication

#endif // PURE_CLIENT_H
