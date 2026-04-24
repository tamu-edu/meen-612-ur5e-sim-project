#include <ur_client_library/example_robot_wrapper.h>
#include <ur_client_library/ur/dashboard_client.h>
#include <ur_client_library/ur/ur_driver.h>
#include <ur_client_library/types.h>
#include <ur_client_library/ur/instruction_executor.h>
#include <zmqpp/zmqpp.hpp>
#include <string>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <cstddef>

#include <iostream>
#include <memory>


#define COMMAND_SIZE 6
#define DATA_SIZE 61
#define MAX_CONTROL 5
#define APPLY_CONTROL true
#define EXTRACT_DATA(name) if (!data_pkg.getData( #name , name ))\
    {\
      std::string error_msg = "Did not find " #name " in data sent from robot. This should not happen!";\
      throw std::runtime_error(error_msg);\
    }

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
const std::string SCRIPT_FILE = "resources2026/external_control.urscript";
const std::string OUTPUT_RECIPE = "resources2026/rtde_output_meen612driver.txt";
const std::string INPUT_RECIPE = "resources2026/rtde_input_recipe.txt";

std::unique_ptr<ExampleRobotWrapper> g_my_robot;
// double robot_timestamp;
// vector6d_t actual_q;
// vector6d_t actual_qd;
// vector6d_t actual_current;
// vector6d_t actual_joint_voltage;
// vector6d_t target_current;
// vector6d_t target_moment;
// vector6d_t target_q;
// vector6d_t target_qd;
// vector6d_t target_qdd;

double timestamp;
vector6d_t target_q;
vector6d_t target_qd;
vector6d_t target_qdd;
vector6d_t target_current;
vector6d_t target_moment;
vector6d_t actual_q;
vector6d_t actual_qd;
vector6d_t actual_current;
vector6d_t actual_current_window;
vector6d_t actual_current_as_torque;
vector6d_t joint_control_output;
vector6d_t actual_TCP_pose;
vector6d_t actual_TCP_speed;
vector6d_t actual_TCP_force;
vector6d_t tcp_offset;
vector6d_t actual_TCP_acceleration;
vector6d_t target_TCP_acceleration;
vector6d_t joint_temperatures;
double actual_execution_time; // ms
double target_execution_time; // ms
int32_t robot_mode;
uint32_t robot_status_bits;
vector6int32_t joint_mode;
int32_t safety_status;
uint32_t safety_status_bits;
vector3d_t actual_tool_accelerometer;
double actual_main_voltage;
double actual_robot_voltage;
double actual_robot_current;
vector6d_t actual_joint_voltage;
uint32_t runtime_state;
vector6d_t ft_raw_wrench;
vector6d_t wrench_calc_from_currents;
double payload;
vector3d_t payload_cog;
vector6d_t payload_inertia;

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
  g_my_robot = std::make_unique<ExampleRobotWrapper>( robot_ip, OUTPUT_RECIPE, 
                                                      INPUT_RECIPE, headless_mode,
                                                      "external_control.urp", SCRIPT_FILE);
  if (!g_my_robot->isHealthy())
  {
    URCL_LOG_ERROR("Something in the robot initialization went wrong. Exiting. Please check the output above.");
    return 1;
  }


  { // New: checks for compatible versions
    auto robot_version = g_my_robot->getUrDriver()->getVersion();
    bool version_supported =
        ((robot_version.major == 5) && (robot_version >= urcl::VersionInformation::fromString("5.25.1"))) ||
        ((robot_version.major >= 10) && (robot_version >= urcl::VersionInformation::fromString("10.11.0")));
    if (!version_supported)
    {
      URCL_LOG_INFO("This direct_torque control example requires a robot with at least version 5.25.1 / 10.12.1. Your "
                    "robot has version %s. Skipping.",
                    robot_version.toString().c_str());
      return 0;
    }
  }
  // --------------- INITIALIZATION END -------------------


  bool first_loop = true;
  URCL_LOG_INFO("Start moving the robot");
  urcl::vector6d_t my_target_torques = { 0, 0, 0, 0, 0, 0 };
  urcl::vector6d_t my_initial_q = { 0, 0, 0, 0, 0, 0 };
  int ndx = 0;

  // Enable when we transition to 10.12.X
  // // Scale each individual joint's friction compensation. This is supported from PolyScope 5.25.1 / PolyScope X 10.12.1
  // // and upwards.
  // urcl::vector6d_t viscous_scale = { 0   .0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  // urcl::vector6d_t coulomb_scale = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  g_my_robot->getUrDriver()->setFrictionCompensation(true); // for version <= 10.11

  // Once RTDE communication is started, we have to make sure to read from the interface buffer, as
  // otherwise we will get pipeline overflows. Therefor, do this directly before starting your main
  // loop.
  g_my_robot->getUrDriver()->startRTDECommunication();

  urcl::rtde_interface::DataPackage data_pkg(g_my_robot->getUrDriver()->getRTDEOutputRecipe());
  if (!g_my_robot->getUrDriver()->getDataPackage(data_pkg))
  {
    URCL_LOG_ERROR("Could not get fresh data package from robot");
    return 1;
  }
  if (!data_pkg.getData("actual_q", actual_q))
  {
    // This throwing should never happen unless misconfigured
    std::string error_msg = "Did not find 'actual_q' in data sent from robot. This should not happen!";
    throw std::runtime_error(error_msg);
  }

  std::stringstream msg;
  msg << "( "<<std::setprecision(12);
  for (int i=0;i<6;i++){
    msg << actual_q[i] <<", ";
  }
  msg << " )";
  URCL_LOG_INFO("Initial joint angles: %s", msg.str().c_str());
  actual_q[5]+=0.01;

  auto instruction_executor = std::make_shared<urcl::InstructionExecutor>(g_my_robot->getUrDriver());
  instruction_executor->optimoveJ(actual_q);

  while (true)
  {
    ndx++;
    // Read latest RTDE package. This will block for a hard-coded timeout (see UrDriver), so the
    // robot will effectively be in charge of setting the frequency of this loop.
    // In a real-world application this thread should be scheduled with real-time priority in order
    // to ensure that this is called in time.
    if (!g_my_robot->getUrDriver()->getDataPackage(data_pkg))
    {
      URCL_LOG_ERROR("Could not get fresh data package from robot");
      return 1;
    }
    EXTRACT_DATA(timestamp)
    EXTRACT_DATA(target_qd)
    EXTRACT_DATA(target_qdd)
    EXTRACT_DATA(target_current)
    EXTRACT_DATA(target_moment)
    EXTRACT_DATA(actual_q)
    EXTRACT_DATA(actual_qd)
    EXTRACT_DATA(actual_current)
    EXTRACT_DATA(actual_current_window)
    EXTRACT_DATA(actual_current_as_torque)
    EXTRACT_DATA(joint_control_output)
    EXTRACT_DATA(actual_TCP_pose)
    EXTRACT_DATA(actual_TCP_speed)
    EXTRACT_DATA(actual_TCP_force)
    EXTRACT_DATA(tcp_offset)
    EXTRACT_DATA(actual_TCP_acceleration)
    EXTRACT_DATA(target_TCP_acceleration)
    EXTRACT_DATA(joint_temperatures)
    EXTRACT_DATA(actual_execution_time)
    EXTRACT_DATA(target_execution_time)
    EXTRACT_DATA(robot_mode)
    EXTRACT_DATA(robot_status_bits)
    EXTRACT_DATA(joint_mode)
    EXTRACT_DATA(safety_status)
    EXTRACT_DATA(safety_status_bits)
    EXTRACT_DATA(actual_tool_accelerometer)
    EXTRACT_DATA(actual_main_voltage)
    EXTRACT_DATA(actual_robot_voltage)
    EXTRACT_DATA(actual_robot_current)
    EXTRACT_DATA(actual_joint_voltage)
    EXTRACT_DATA(runtime_state)
    EXTRACT_DATA(ft_raw_wrench)
    EXTRACT_DATA(wrench_calc_from_currents)
    EXTRACT_DATA(payload)
    EXTRACT_DATA(payload_cog)
    EXTRACT_DATA(payload_inertia)

    std::array<double,DATA_SIZE> state_data;
    state_data[0] = timestamp;
    for (int i=0; i<6;i++){
      state_data[1+i] = actual_q[i];
      state_data[7+i] = actual_qd[i];
      state_data[13+i] = actual_current[i];
      state_data[19+i] = actual_joint_voltage[i];
      state_data[25+i] = my_target_torques[i];
      state_data[31+i] = target_current[i];
      state_data[37+i] = target_moment[i];
      state_data[43+i] = target_q[i];
      state_data[49+i] = target_qd[i];
      state_data[55+i] = target_qdd[i];
    }
    std::array<double,COMMAND_SIZE> in_data = synch.update(state_data);

    if (first_loop){
      my_initial_q = actual_q; // always start with current position as the target.
      first_loop = false;
    }

    if (ndx%500==0){
      std::stringstream msg;
      msg<<"[ "<<std::setprecision(6);
      for (int i=0;i<6;i++){
        msg << in_data[i] << "( abs " << ((abs(in_data[i]) >  MAX_CONTROL) ? ">" : "<");
        msg << MAX_CONTROL << ")"<<", ";
      }
      URCL_LOG_INFO("Raw torque command was %s", msg.str().c_str());
    }

    for (int i=0;i<6;i++){
      if(!std::isfinite(in_data[i])){
        std::string error_msg = "Control is not finite. This should not happen!";
        throw std::runtime_error(error_msg);
      }
      if( in_data[i] >  MAX_CONTROL){in_data[i] =  MAX_CONTROL;}
      if( in_data[i] < -MAX_CONTROL){in_data[i] = -MAX_CONTROL;}
      if(APPLY_CONTROL){
        my_target_torques[i] = in_data[i];
      }
    }

    if (ndx%500==0){
      std::stringstream msg;
      msg<<"[ "<<std::setprecision(12);
      for (int i=0;i<6;i++){
        msg << in_data[i] << "( abs " << ((abs(in_data[i]) >  MAX_CONTROL) ? ">" : "<");
        msg << MAX_CONTROL << ")"<<", ";
      }
      URCL_LOG_INFO("Adjusted torque command was %s", msg.str().c_str());
      URCL_LOG_INFO("data_pkg:\n%s", data_pkg.toString().c_str());
    }

    // joint_target[5] += increment;
    // Setting the RobotReceiveTimeout time is for example purposes only. This will make the example running more
    // reliable on non-realtime systems. Use with caution in productive applications.
    bool ret = g_my_robot->getUrDriver()->writeJointCommand(my_target_torques, comm::ControlMode::MODE_TORQUE,
                                                            RobotReceiveTimeout::millisec(100));
    if (!ret)
    {
      URCL_LOG_ERROR("Could not send joint command. Make sure that the robot is in remote control mode and connected "
                     "with a network cable.");
      return 1;
    }
    URCL_LOG_DEBUG("data_pkg:\n%s", data_pkg.toString().c_str());
  }
  g_my_robot->getUrDriver()->stopControl();
  URCL_LOG_INFO("Movement done");
  return 0;
}
