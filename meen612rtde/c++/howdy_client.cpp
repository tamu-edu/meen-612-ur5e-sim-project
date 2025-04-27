//  Howdy World client
#include <zmqpp/zmqpp.hpp>
#include <string>
#include <iostream>
#include <sstream>

// using namespace std;
// bool ZMQ_NOBLOCK = true;
#define COMMAND_SIZE 6
#define DATA_SIZE 25

// class ZmqBinarySynchA:
// class ZmqBinarySynchA
// {
// public:
//   ZmqBinarySynchA(std::string& bindport, std::string& connectport);
//   std::array<double,6>& update(std::array<double, 12>& data_in);
//   int get_count(void){return my_count;}
// private:
//   zmqpp::context context;
//   std::array<double, 12> data_in;
//   std::array<double, 6> data_out;
//   int my_count;
//   zmqpp::socket socketA;
//   zmqpp::socket socketB;
// };
// //     def __init__(self, bindport="tcp://*:5557", connectport="tcp://localhost:5558"):
// //         self.context = zmq.Context()
// //         self.socketA = self.context.socket(zmq.PUB)
// //         self.socketA.bind(bindport)

// //         self.socketB = self.context.socket(zmq.SUB)
// //         self.socketB.setsockopt(zmq.SUBSCRIBE, b'B')
// //         self.socketB.connect(connectport)

// //         self.my_count = -42
// //         self.data_out = None
// ZmqBinarySynchA::ZmqBinarySynchA(std::string& bindport, std::string& connectport)
//     : context()
//     , data_in()
//     , data_out()
//     , my_count(-42)
//     , socketA(context, zmqpp::socket_type::pub)
//     , socketB(context, zmqpp::socket_type::sub)
// {
//   data_in.fill(42.0);
//   data_out.fill(1337.0);
//   // socketA = zmqpp::socket(context, zmqpp::socket_type::pub);
//   socketA.bind(bindport);

//   // socketB = zmqpp::socket(context, zmqpp::socket_type::sub);
//   socketB.connect(connectport);
//   socketB.subscribe("B");
// }


// //     def update(self, data_in):
// //         """ read all messages, then send data. """
// //         while True:
// //             try: 
// //                 message = self.socketB.recv(zmq.NOBLOCK)
// //                 self.my_count=int(message[1:4])
// //                 self.data_out = np.frombuffer(message[4:])
// //             except zmq.error.Again:
// //                 break
// //         self.socketA.send(b"A%03d%s"%(self.my_count, data_in.tobytes()))

// //         return self.data_out
// std::array<double,6>& ZmqBinarySynchA::update(std::array<double, 12>& data_in){
//   /* Read all messages, then send data. */
//   bool keep_reading = true;
//   bool message_fresh = false;
//   int num_msg = 0;
//   while(keep_reading){
//     zmqpp::message_t message_in;
//     message_fresh = socketB.receive(message_in, ZMQ_NOBLOCK);
//     if (!message_fresh) break;
//     std::string res;
//     message_in>>res;
//     std::cout<<res<<std::endl;
//     keep_reading = message_fresh;
    
//     // res.substr(a,b)
//     uint start_index = res.find(",")+1;
//     uint stop_index = res.substr(start_index,res.length()).find(",");
//     my_count = std::stoi(res.substr(start_index,stop_index));

//     for (uint i=0;i<6;i++){
//       start_index = stop_index+1;
//       stop_index = res.substr(start_index, res.length()).find(",");
//       data_out[i] = std::stod(res.substr(start_index, stop_index));
//       // message_in >> data_out[i];
//     }
//   }
//   std::stringstream message_out;
//   message_out << "A,"<< my_count<<",";
//   for (uint i=0;i<12;i++){
//     message_out << data_in[i];
//     if (i<11) message_out<<",";
//   }
//   zmqpp::message msg_out;
//   msg_out << message_out.str();
//   socketA.send(msg_out);

//   return data_out;
// }

class ZmqBinarySynchA
{
public:
  ZmqBinarySynchA(std::string& bindport, std::string& connectport);
  std::array<double,COMMAND_SIZE>& update(std::array<double, DATA_SIZE>& data_in);
  int get_count(void){return my_count;}
private:
  zmqpp::context context;
  std::array<double, DATA_SIZE> data_in;
  std::array<double, COMMAND_SIZE> data_out;
  int my_count;
  zmqpp::socket socketA;
  zmqpp::socket socketB;
};

ZmqBinarySynchA::ZmqBinarySynchA(std::string& bindport, std::string& connectport)
    : context()
    , data_in()
    , data_out()
    , my_count(-42)
    , socketA(context, zmqpp::socket_type::pub)
    , socketB(context, zmqpp::socket_type::sub)
{
  data_in.fill(0.0);
  data_out.fill(0.0);
  socketA.bind(bindport);

  socketB.connect(connectport);
  socketB.subscribe("B");
}

std::array<double,COMMAND_SIZE>& ZmqBinarySynchA::update(std::array<double, DATA_SIZE>& data_in){
  /* Read all messages, then send data. */
  bool keep_reading = true;
  bool message_fresh = false;
  int num_msg = 0;
  while(keep_reading){
    zmqpp::message_t message_in;
    message_fresh = socketB.receive(message_in, ZMQ_NOBLOCK);
    if (!message_fresh) break;
    std::string res;
    message_in>>res;
    std::cout<<res<<std::endl;
    keep_reading = message_fresh;
    
    uint start_index = res.find(",")+1;
    uint stop_index = res.substr(start_index,res.length()).find(",");
    my_count = std::stoi(res.substr(start_index,stop_index));

    for (uint i=0;i<COMMAND_SIZE;i++){
      start_index = stop_index+1;
      stop_index = res.substr(start_index, res.length()).find(",");
      data_out[i] = std::stod(res.substr(start_index, stop_index));
    }
  }
  std::stringstream message_out;
  message_out << "A,"<< my_count<<",";
  for (uint i=0;i<DATA_SIZE;i++){
    message_out << data_in[i];
    if (i<DATA_SIZE-1) message_out<<",";
  }
  zmqpp::message msg_out;
  msg_out << message_out.str();
  socketA.send(msg_out);

  return data_out;
}

int main(int argc, char *argv[]) {
  std::string bindport = "ipc:///tmp/feeds/30";
  std::string connectport = "ipc:///tmp/feeds/31";
  ZmqBinarySynchA synch(bindport, connectport);

  std::array<double,25> fake_data = {100., 200.,300.,400.,5.,6.,7.,8.,9.,10.,11.,12.,13,14,15,16,17,18,19,20,21,22,23,24,25};
  for (int i=0;i<100; i++){
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    std::array<double,6> in_data = synch.update(fake_data);
    fake_data[3] = in_data[3];
    std::cout<<"count: "<<synch.get_count()<<", data: ";
    for (int j =0; j<6; j++){
      std::cout<<in_data[j];
      if(j<5) std::cout<<", ";
    }
    std::cout<<std::endl;
  }
}

/*int main(int argc, char *argv[]) {
  // const string endpoint = "tcp://localhost:4242";
  const std::string endpoint = "ipc:///tmp/feeds/42";

  // initialize the 0MQ context
  zmqpp::context context;

  // generate a push socket
  zmqpp::socket_type type = zmqpp::socket_type::push;
  zmqpp::socket socket (context, type);

  // open the connection
  std::cout << "Opening connection to " << endpoint << "..." << std::endl;
  socket.connect(endpoint);

  // send a message
  std::cout << "Sending text and a number..." << std::endl;
  zmqpp::message message;
  // compose a message from a string and a number
  message << "Howdy, World!" << 42;
  socket.send(message);
  
  std::cout << "Sent message." << std::endl;
  std::cout << "Finished." << std::endl;
}*/

