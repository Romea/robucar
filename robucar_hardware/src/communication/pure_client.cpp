#include <stdio.h>
#include <string.h>
#include <iostream>
#include "robucar_hardware/communication/pure_client.hpp"

namespace robucar_communication {

using namespace std;

// prototype
bool checkMessageLength (const size_t bytesRead, const size_t minLen);

//
// static instance chosen for singleton implementation
// --------------------------
// advantages: thread safe, automatic deallocation
//
// disadvantages: memory reserved at startup (even if no instance is required),
// deallocation of global objects in undefined order
//
PureClient& PureClient::getInstance(std::string host, unsigned int port) {
  static PureClient instance(host, port);
  return instance;
}

//
// private constructor, set up networking
//
PureClient::PureClient(const string host, const unsigned int port) :
    resolver(io_service), query(udp::v4(), host, ""), socket(io_service)
{
  try
  {
#ifndef NDEBUG
    cout << "####### CONSTRUCTING PURE CLIENT INSTANCE #######" << endl;
#endif
    receiver = *resolver.resolve(query);
    receiver.port(port);
    socket.open(udp::v4());
    startThreadedReception();
  }
  catch (exception& e) {
#ifndef NDEBUG
    cout << "Exception in PureClient: " << e.what() << endl;
#endif
  }
}

//
// private destructor, finalize networking
//
PureClient::~PureClient()
{
  io_service.stop();
  socket.close();
#ifndef NDEBUG
  printf ("\nPureClient: Socket closed\n");
#endif
  run=false;
  if (thrd) {
    thrd->join();
    delete thrd;
#ifndef NDEBUG
    cout << "PureClient: Reception thread stopped" << endl;
#endif
  }
}

//
//
//
bool PureClient::socketValid ()
{
  return socket.is_open();
}

 /*
  * looks in message buffer for the most recent message for a specified service (target)
  *
  *
  */
ResponsePtr PureClient::findResponse
    (const uint8_t id, const uint8_t action,
     const uint16_t target, unsigned int waitMs)
{

  ResponsePtr p;
  deque<ResponsePtr>::iterator it;

  #ifndef NDEBUG
    printf("Looking for response (id=%u,action=%u,target=%u)\n", id, action, target);
  #endif
  for (int i=0; i<3; i++){

    mutex.lock();
    for (it=responseMessages.begin(); it<responseMessages.end(); it++)
    {
      if ((*it)->matches(id, action, target)) {
        mutex.unlock();
        #ifndef NDEBUG
        printf("PureClient: Found response (id=%u,action=%u,target=%u) in buffer of %d messages\n",
          id, action, target, int(responseMessages.size()));
        #endif
        // remeber pointer, I like smart pointers
        p = (*it);
        // delete buffer entry
        responseMessages.erase(it);
        return p;
      } else {
        #ifndef NDEBUG
          printf("--- is (%u %u %u) instead of (%u %u %u)\n", (*it)->header.id, (*it)->header.action,
            (*it)->header.target, id, action, target);
        #endif
      }
    }
    mutex.unlock();
    boost::this_thread::sleep(boost::posix_time::milliseconds(waitMs/3));
  }
#ifndef NDEBUG
  printf("PureClient: Response message not found (id=%u,action=%u,target=%d)\n", id, action, target);
#endif
  return p;
}

/*
 * performs first GET for the directory, then QUERY for every entry in order to
 * receive a description
 *
 */
void PureClient::getDirectory()
{
  unsigned int len=0;
  uint8_t sndBuf[BUF_SIZE];
  string error;


  //
  // STEP 1: REQUEST DIRECTORY
  //
  PureRequestHeader reqHeader_get (1, Pure::ACTION_GET, Pure::TARGET_DIR);
  ResponsePtr response;

  // request header (GET)
  len += reqHeader_get.toBuffer(sndBuf);

  if (!send(sndBuf, len)) {
#ifndef NDEBUG
    printf("PureClient: Error sending directory request\n");
#endif
    return;
  }

  // get answer
  response = findResponse(1, Pure::ACTION_GET, Pure::TARGET_DIR);
  if (!response) {
#ifndef NDEBUG
    printf("PureClient: No answer to directory request\n");
#endif
    return;
  }

  // extract directory entries
  if (response->dataLength%4==0)
  {
   int nb_entries = (static_cast<int>(response->dataLength))/4;
   // convert to directory
   directory.fromBuffer (response->dataBuffer, nb_entries);
  }
  else
  {
#ifndef NDEBUG
    cout << "PureClient Error: Corrupted directory data received" << endl;
#endif
  }

  //
  // STEP 2: REQUEST DESCRIPTIONS
  //
  PureRequestHeader reqHeader_query (1, Pure::ACTION_QUERY, Pure::TARGET_DIR);

  // request decription for every entry
  for (vector<DirEntry*>::iterator it=directory.dir.begin(); it<directory.dir.end(); it++)
  {
    // get instance of an entry
    uint16_t instance;
    instance = (*it)->instance;

    // request header (QUERY)
    len = reqHeader_query.toBuffer(sndBuf);

    // add it to send buffer
    memcpy (sndBuf+len, &instance, 2); // do not increment in order to overwrite it later
    //ret = socket.send_to (boost::asio::buffer(sndBuf, len+2), receiver);

    if (!send(sndBuf, len+2)) {
#ifndef NDEBUG
      printf("PureClient: Error sending service description request\n");
#endif
      return;
    }

    // read response
    {
      response = findResponse(1, Pure::ACTION_QUERY, Pure::TARGET_DIR);
      if (!response) {
#ifndef NDEBUG
        printf("PureClient: No answer to directory request (instance %d)\n", instance);
#endif
        return;
      }

      // check for error
      if (response->header.error != Pure::ERROR_SUCCESS) {
#ifndef NDEBUG
        cout << "PureClient: Error reported in response to directory description" <<
            " request: " << response->header.error << endl;
#endif
      } else {

        // add \0 character to terminate string
        response->dataBuffer[response->dataLength] = '\0';
        #ifndef NDEBUG
          printf("Received string: %s\n", response->dataBuffer);
        #endif
        (*it)->description = reinterpret_cast<char*>(response->dataBuffer);
        #ifndef NDEBUG
          printf("Received string: %s\n", (*it)->description.c_str());
        #endif
      }
    }
  }

}

//
// output directory to console
//
void PureClient::printDirectory() {
  directory.print();
}

//
// send a notification request for 'instance', with period 'period' and registers
// a handler to be called back on reception
//
void PureClient::requestNotification
    (const uint16_t instance, const uint8_t period, HandleFun handle)
{

  if (sendNotificationRequest(Pure::ACTION_INSERT, instance, period))
    // TODO change to smart pointers
    notifications.entries.push_back(new NotificationEntry(instance,handle));

}

//
// send a notification request for 'instance', with period 'period' and registers
// a handler to be called back on reception
//
NotificationEntry* PureClient::requestNotification
    (uint16_t instance, uint8_t period, int historySize)
{
  if (sendNotificationRequest(Pure::ACTION_INSERT, instance, period))
  {
    // TODO change to smart pointers
    NotificationEntry* ne = new NotificationEntry(instance,static_cast<unsigned int>(historySize));
    notifications.entries.push_back(ne);
    return ne;
  }
  return NULL;
}

int PureClient::requestNotificationStop (const uint16_t instance)
{
#ifndef NDEBUG
  printf("Requesting notification stop (target=%u)\n", instance);
#endif
  return (sendNotificationRequest(Pure::ACTION_DELETE, instance, 0));
}


bool PureClient::sendNotificationRequest (const uint8_t action,
                                          const uint16_t instance, const uint8_t period)
{
  uint8_t sndBuf[BUF_SIZE];
  ResponsePtr response;

#ifndef NDEBUG
  printf("Sending notification request (target=%u)\n", instance);
#endif

  PureRequestHeader reqHeader(3, action, Pure::TARGET_NOTIFICATION);
  NotificationRequest notRequest(instance, period);

  unsigned int len = reqHeader.toBuffer(sndBuf);
  len += notRequest.toBuffer(sndBuf+len);

  // send request
  if (!send(sndBuf, len)) {
    // error
#ifndef NDEBUG
    printf("PureClient Error: Failed sending notification request (target=%u)\n", instance);
#endif
    return false;
  }

  response = findResponse(3, action, Pure::TARGET_NOTIFICATION);
  if (!response) {
#ifndef NDEBUG
    printf("PureClient: No answer to notification request (target=%u)\n", instance);
#endif
    return false;
  }

  if (response->header.error != Pure::ERROR_SUCCESS)
  {
#ifndef NDEBUG
    printf("PureClent Error: Received error as response to notification request (target=%u): 0x%X\n", instance,
        response->header.error);
#endif
  }

  return true;
}

bool PureClient::sendRequest(uint8_t id, uint8_t action, uint16_t target)
{
  uint8_t sndBuf[BUF_SIZE];
  PureRequestHeader reqHeader(id, action, target);
  unsigned int len = reqHeader.toBuffer(sndBuf);

  // send request
  if (!send(sndBuf, len)) {
    // error
#ifndef NDEBUG
    cout << "PureClient Error: Failed sending request" << endl;
#endif
    return false;
  }
  return true;
}


//
// treat received message, three cases:
//  1) Notification -> call-back
//  2) Error        -> message
//  3) Response     -> create PureResponse instance and enqueue
//
void PureClient::parseMessage (const size_t bytes_received)
{

  // check length
  if (!checkMessageLength(bytes_received, Pure::LENGTH_RESPONSE)) {
#ifndef NDEBUG
    cout << "PureClient Error: response too short!" << endl;
#endif
    return;
  }


  // create response header, even if it could be a notification or an error
  //(whose header is different), but thus we can check the ID
  PureResponseHeader respHeader;
  size_t len = respHeader.fromBuffer(recvBuf);

  // test special cases first, reread header then
  switch (respHeader.id) {

  case Pure::ID_NOTIFICATION:
    {
      // read notification header
      NotificationOutboundHeader notHeader;
      len = notHeader.fromBuffer(recvBuf);
     // printf("received notification target=%d time=%lld\n",
             //notHeader.target, notHeader.timestamp);

      // find and call handler of this instance
      vector<NotificationEntry*>::iterator it;

      for (it=notifications.entries.begin(); it<notifications.entries.end(); it++)
      {
        if ((*it)->instance == notHeader.target) {
          //printf("found instance %d in vector of %d entries\n", notHeader.target, notifications.entries.size());
          // call-back

          if ((*it)->hasHandle)
            (*it)->handle(recvBuf+len, bytes_received-len);
          else {
            (*it)->enqueue
                (OutboundNotificationPtr(new OutboundNotification
                  (notHeader,
                   recvBuf+Pure::LENGTH_OUTB_NOTIFICATION,
                   bytes_received-Pure::LENGTH_OUTB_NOTIFICATION))
                 );
          }
          break;
        }
      }
      if (it>=notifications.entries.end()) {
        //cout << "PureClient: Error: No callback function found for notification target <"
          //  << notHeader.target << ">" << endl;
      }
    }
    break;

  case Pure::ID_ERROR:
#ifndef NDEBUG
    printf("PureClient: Received error message: code=%d/%s\n",
           respHeader.error, Pure::errorMessage(respHeader.error).c_str());
#endif
    break;

  default:
  {
    // in this case: response to request

    // copy data
    // administrative message -> enqueue
    mutex.lock();
    responseMessages.push_front(ResponsePtr(new PureResponse(recvBuf, bytes_received)));
    mutex.unlock();

  }
  break;
  }
}

 /*
  *
  * RECEPTION (asynchronous)
  *
  */

//
// callback function to process received data
//
void PureClient::handle_receive_from
    (const boost::system::error_code& /*error*/, size_t bytes_recvd)
{
#ifndef NDEBUG
  printf("Received %d bytes\n", int(bytes_recvd));
  for (size_t i=0; i<bytes_recvd; i++)
    printf(" 0x%02X", recvBuf[i]);
  printf("\n");
#endif
  parseMessage(bytes_recvd);
  startAsyncReception();

}

//
// registers listener, has to be called after every data reception
//
void PureClient::startAsyncReception()
{
  socket.async_receive_from(
      boost::asio::buffer(recvBuf, 4096), sender,
      boost::bind(&PureClient::handle_receive_from, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
}

//
// starts reception thread, called by constructor at creation
//
bool PureClient::startThreadedReception()
{
#ifndef NDEBUG
  cout << "PureClient: Starting reception thread" << endl;
#endif
  startAsyncReception();
  thrd = new boost::thread(boost::bind(&PureClient::runReception, this));
  return true;
}

//
// reception thread, maintains the boost ioservice
//
void PureClient::runReception()
{
  run = true;
  boost::system::error_code error;
#ifndef NDEBUG
  cout << "PureClient: Running IO service" << endl;
#endif
  do { io_service.run( error ); } while( !error && run );
}

//
// was ment to be more complicated, thus not really necessary ...
//
bool checkMessageLength (const size_t bytesRead, const size_t minLen)
{ return (bytesRead >= minLen); }

/*
 *
 * SENDING (synchronous)
 *
 */

//
// supports only a single endpoint
//
size_t PureClient::send (const uint8_t *buf, const size_t len)
{
  size_t ret = socket.send_to(boost::asio::buffer(buf, len), receiver);

  if (ret==len) {
  #ifndef NDEBUG
    cout << "Sent buffer of " << ret << " bytes" << endl;
    for (size_t i=0; i<ret; i++)
      printf(" 0x%02X", buf[i]);
    printf("\n");
  #endif
  }
  return ret;
}

//
// send a inbound notification to the server
// returns size of data bytes send (excluding header!)
//
size_t PureClient::sendNotification
    (const uint16_t target, const uint8_t *buf, const size_t len)
{
  // prepare buffer
  uint8_t sndBuf[BUF_SIZE];
  NotificationInboundHeader nih(target);
  size_t len_header = nih.toBuffer(sndBuf);
  //printf("debug1: 0x%X 0x%X 0x%X\n", sndBuf[0], sndBuf[1], sndBuf[2]);
  memcpy(sndBuf+len_header, buf, len);
  //printf("debug2: 0x%X 0x%X 0x%X\n", sndBuf[0], sndBuf[1], sndBuf[2]);

  // send
  return ( send(sndBuf, len+len_header) - len_header );
}

} //end namespace robucar_communication
