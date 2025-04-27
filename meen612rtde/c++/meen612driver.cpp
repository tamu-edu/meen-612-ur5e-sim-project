#include <ur_client_library/example_robot_wrapper.h>
#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/types.h>
#include <zmqpp/zmqpp.hpp>
#include <string>
#include <sstream>
#include <cmath>
#include <iomanip>

#include <iostream>
#include <memory>


#define COMMAND_SIZE 6
#define DATA_SIZE 61
#define MAX_CONTROL 0.005
#define APPLY_CONTROL true

// class ZmqBinarySynchA:
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
    // std::cout<<res<<std::endl;
    keep_reading = message_fresh;
    
    uint start_index = res.find(",")+1;
    uint stop_index = start_index+res.substr(start_index).find(",");
    my_count = std::stoi(res.substr(start_index,stop_index-start_index));

    // std::cout<<"Message parsing:"<<std::endl;

    for (uint i=0;i<COMMAND_SIZE;i++){
      start_index = stop_index+1;
      stop_index = start_index+res.substr(start_index).find(",");
      data_out[i] = std::stod(res.substr(start_index, stop_index-start_index));
      // std::cout<<"\t"<<i<<", "<<start_index<<", "<<stop_index<<", '"<<res.substr(start_index, stop_index-start_index)<<"', i.e., "<<std::setprecision(12)<<data_out[i]<<std::endl;
    }
  }
  std::stringstream message_out;
  message_out << "A,"<< my_count<<","<<std::setprecision(12);
  for (uint i=0;i<DATA_SIZE;i++){
    message_out << data_in[i];
    if (i<DATA_SIZE-1) message_out<<",";
  }
  zmqpp::message msg_out;
  msg_out << message_out.str();
  socketA.send(msg_out);

  return data_out;
}


using namespace urcl;

// In a real-world example it would be better to get those values from command line parameters / a
// better configuration system such as Boost.Program_options
const std::string DEFAULT_ROBOT_IP = "192.168.3.101";
const std::string SCRIPT_FILE = "resources/external_control.urscript";
const std::string OUTPUT_RECIPE = "resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "resources/rtde_input_recipe.txt";

std::unique_ptr<ExampleRobotWrapper> g_my_robot;
double robot_timestamp;
vector6d_t actual_q;
vector6d_t actual_qd;
vector6d_t actual_current;
vector6d_t actual_joint_voltage;
vector6d_t target_current;
vector6d_t target_moment;
vector6d_t target_q;
vector6d_t target_qd;
vector6d_t target_qdd;

int main(int argc, char* argv[])
{
  urcl::setLogLevel(urcl::LogLevel::INFO);

  std::string bindport = "ipc:///tmp/feeds/30";
  std::string connectport = "ipc:///tmp/feeds/31";
  ZmqBinarySynchA synch(bindport, connectport);

  // Parse the ip arguments if given
  std::string robot_ip = DEFAULT_ROBOT_IP;
  if (argc > 1)
  {
    robot_ip = std::string(argv[1]);
  }

  bool headless_mode = true;
  g_my_robot = std::make_unique<ExampleRobotWrapper>(robot_ip, OUTPUT_RECIPE, INPUT_RECIPE, headless_mode,
                                                     "external_control.urp");
  if (!g_my_robot->isHealthy())
  {
    URCL_LOG_ERROR("Something in the robot initialization went wrong. Exiting. Please check the output above.");
    return 1;
  }
  // --------------- INITIALIZATION END -------------------


  bool first_loop = true;
  URCL_LOG_INFO("Start moving the robot");
  urcl::vector6d_t joint_target = { 0, 0, 0, 0, 0, 0 };
  int ndx = 0;

  // Once RTDE communication is started, we have to make sure to read from the interface buffer, as
  // otherwise we will get pipeline overflows. Therefor, do this directly before starting your main
  // loop.
  g_my_robot->getUrDriver()->startRTDECommunication();
  while (true)
  {
    ndx++;
    // Read latest RTDE package. This will block for a hard-coded timeout (see UrDriver), so the
    // robot will effectively be in charge of setting the frequency of this loop.
    // In a real-world application this thread should be scheduled with real-time priority in order
    // to ensure that this is called in time.
    std::unique_ptr<rtde_interface::DataPackage> data_pkg = g_my_robot->getUrDriver()->getDataPackage();
    if (!data_pkg)
    {
      URCL_LOG_WARN("Could not get fresh data package from robot");
      return 1;
    }

    if (!data_pkg->getData("timestamp", robot_timestamp))
    {
      std::string error_msg = "Did not find 'timestamp' in data sent from robot. This should not happen!";
      throw std::runtime_error(error_msg);
    }

    if (!data_pkg->getData("actual_q", actual_q))
    {
      std::string error_msg = "Did not find 'actual_q' in data sent from robot. This should not happen!";
      throw std::runtime_error(error_msg);
    }

    if (!data_pkg->getData("actual_qd", actual_qd))
    {
      std::string error_msg = "Did not find 'actual_qd' in data sent from robot. This should not happen!";
      throw std::runtime_error(error_msg);
    }

    if (!data_pkg->getData("actual_current", actual_current))
    {
      std::string error_msg = "Did not find 'actual_current' in data sent from robot. This should not happen!";
      throw std::runtime_error(error_msg);
    }

    if (!data_pkg->getData("actual_joint_voltage", actual_joint_voltage))
    {
      std::string error_msg = "Did not find 'actual_joint_voltage' in data sent from robot. This should not happen!";
      throw std::runtime_error(error_msg);
    }
    if (!data_pkg->getData("target_current", target_current))
    {
      std::string error_msg = "Did not find 'target_current' in data sent from robot. This should not happen!";
      throw std::runtime_error(error_msg);
    }
    if (!data_pkg->getData("target_moment", target_moment))
    {
      std::string error_msg = "Did not find 'target_moment' in data sent from robot. This should not happen!";
      throw std::runtime_error(error_msg);
    }
    if (!data_pkg->getData("target_q", target_q))
    {
      std::string error_msg = "Did not find 'target_q' in data sent from robot. This should not happen!";
      throw std::runtime_error(error_msg);
    }
    if (!data_pkg->getData("target_qd", target_qd))
    {
      std::string error_msg = "Did not find 'target_qd' in data sent from robot. This should not happen!";
      throw std::runtime_error(error_msg);
    }
    if (!data_pkg->getData("target_qdd", target_qdd))
    {
      std::string error_msg = "Did not find 'target_qdd' in data sent from robot. This should not happen!";
      throw std::runtime_error(error_msg);
    }

    std::array<double,DATA_SIZE> state_data;
    state_data[0] = robot_timestamp;
    for (int i=0; i<6;i++){
      state_data[1+i] = actual_q[i];
      state_data[7+i] = actual_qd[i];
      state_data[13+i] = actual_current[i];
      state_data[19+i] = actual_joint_voltage[i];
      state_data[25+i] = joint_target[i];
      state_data[31+i] = target_current[i];
      state_data[37+i] = target_moment[i];
      state_data[43+i] = target_q[i];
      state_data[49+i] = target_qd[i];
      state_data[55+i] = target_qdd[i];
    }
    std::array<double,COMMAND_SIZE> in_data = synch.update(state_data);

    if (first_loop){
      joint_target = actual_q; // always start with current position as the target.
      first_loop = false;
    }

    if (ndx%100==0){
      std::stringstream msg;
      msg<<"[ "<<std::setprecision(6);
      for (int i=0;i<6;i++){
        msg << in_data[i] << "(" << ((in_data[i] >  MAX_CONTROL) ? ">" : "<");
        msg << "0.0005)"<<", ";
      }
      URCL_LOG_INFO("Raw position command was %s", msg.str().c_str());
    }

    for (int i=0;i<6;i++){
      if(!std::isfinite(in_data[i])){
        std::string error_msg = "Control is not finite. This should not happen!";
        throw std::runtime_error(error_msg);
      }
      if( in_data[i] >  MAX_CONTROL){in_data[i] =  MAX_CONTROL;}
      if( in_data[i] < -MAX_CONTROL){in_data[i] = -MAX_CONTROL;}
      if(APPLY_CONTROL){
        joint_target[i] = actual_q[i]+in_data[i]; // apply control as an augmentation to target position.
      }
    }

    if (ndx%100==0){
      std::stringstream msg;
      msg<<"[ "<<std::setprecision(12);
      for (int i=0;i<6;i++){
        msg << in_data[i] << "(" << ((in_data[i] >  MAX_CONTROL) ? ">" : "<");
        msg << "0.0005)"<<", ";
      }
      URCL_LOG_INFO("Adjusted position command was %s", msg.str().c_str());
    }

    // joint_target[5] += increment;
    // Setting the RobotReceiveTimeout time is for example purposes only. This will make the example running more
    // reliable on non-realtime systems. Use with caution in productive applications.
    bool ret = g_my_robot->getUrDriver()->writeJointCommand(joint_target, comm::ControlMode::MODE_SERVOJ,
                                                            RobotReceiveTimeout::millisec(100));
    if (!ret)
    {
      URCL_LOG_ERROR("Could not send joint command. Is the robot in remote control?");
      return 1;
    }
    URCL_LOG_DEBUG("data_pkg:\n%s", data_pkg->toString().c_str());
  }
  g_my_robot->getUrDriver()->stopControl();
  URCL_LOG_INFO("Movement done");
  return 0;
}
