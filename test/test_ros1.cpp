#include "doctest.h"

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>

#include "rosx_introspection/ros_parser.hpp"

#include "rosx_introspection/ros_utils/ros1_helpers.hpp"

using namespace RosMsgParser;

TEST_CASE("Parse ROS1 [JointState]")
{
  ParsersCollection<ROS_Deserializer> parser;
  parser.registerParser("joint_state",
                        GetMessageType<sensor_msgs::JointState>(),
                        GetMessageDefinition<sensor_msgs::JointState>());

  sensor_msgs::JointState joint_state;

  joint_state.header.seq = 2016;
  joint_state.header.stamp.sec  = 1234;
  joint_state.header.stamp.nsec = 567*1000*1000;
  joint_state.header.frame_id = "pippo";

  joint_state.name.resize( 3 );
  joint_state.position.resize( 3 );
  joint_state.velocity.resize( 3 );
  joint_state.effort.resize( 3 );

  std::string names[3];
  names[0] = ("hola");
  names[1] = ("ciao");
  names[2] = ("bye");

  for (int i=0; i<3; i++)
  {
    joint_state.name[i] = names[i];
    joint_state.position[i]= 10+i;
    joint_state.velocity[i]= 30+i;
    joint_state.effort[i]= 50+i;
  }

  std::vector<uint8_t> buffer = BuildMessageBuffer(joint_state);

  auto flat_container = parser.deserialize("joint_state", Span<uint8_t>(buffer) );

  for(auto&it: flat_container->value) {
    std::cout << it.first << " >> " << it.second.convert<double>() << std::endl;
  }

  for(auto&it: flat_container->name) {
    std::cout << it.first << " >> " << it.second << std::endl;
  }

  CHECK( flat_container->value[0].first.toStdString() == ("joint_state/header/seq"));
  CHECK( flat_container->value[0].second.convert<int>() ==  2016 );

  CHECK( flat_container->value[1].first.toStdString() == ("joint_state/header/stamp"));
  CHECK( flat_container->value[1].second.convert<double>() == 1234.567  );
  auto time = flat_container->value[1].second.convert<Time>();
  CHECK( time.sec == joint_state.header.stamp.sec  );
  CHECK( time.nsec == joint_state.header.stamp.nsec  );

  CHECK( flat_container->value[2].first.toStdString() == ("joint_state/position[0]"));
  CHECK( flat_container->value[2].second.convert<int>() == 10 );
  CHECK( flat_container->value[3].first.toStdString() == ("joint_state/position[1]"));
  CHECK( flat_container->value[3].second.convert<int>() == 11 );
  CHECK( flat_container->value[4].first.toStdString() == ("joint_state/position[2]"));
  CHECK( flat_container->value[4].second.convert<int>() == 12 );

  CHECK( flat_container->value[5].first.toStdString() == ("joint_state/velocity[0]"));
  CHECK( flat_container->value[5].second.convert<int>() == 30 );
  CHECK( flat_container->value[6].first.toStdString() == ("joint_state/velocity[1]"));
  CHECK( flat_container->value[6].second.convert<int>() == 31 );
  CHECK( flat_container->value[7].first.toStdString() == ("joint_state/velocity[2]"));
  CHECK( flat_container->value[7].second.convert<int>() == 32 );

  CHECK( flat_container->value[8].first.toStdString() == ("joint_state/effort[0]"));
  CHECK( flat_container->value[8].second.convert<int>() == 50 );
  CHECK( flat_container->value[9].first.toStdString() == ("joint_state/effort[1]"));
  CHECK( flat_container->value[9].second.convert<int>() == 51 );
  CHECK( flat_container->value[10].first.toStdString() == ("joint_state/effort[2]"));
  CHECK( flat_container->value[10].second.convert<int>() == 52 );


  CHECK( flat_container->name[0].first.toStdString() == ("joint_state/header/frame_id"));
  CHECK( flat_container->name[0].second == ("pippo") );

  CHECK( flat_container->name[1].first.toStdString() == ("joint_state/name[0]"));
  CHECK( flat_container->name[1].second == ("hola") );

  CHECK( flat_container->name[2].first.toStdString() == ("joint_state/name[1]"));
  CHECK( flat_container->name[2].second == ("ciao") );

  CHECK( flat_container->name[3].first.toStdString() == ("joint_state/name[2]"));
  CHECK( flat_container->name[3].second == ("bye") );
}

TEST_CASE("Parse ROS1 JSON [JointState]")
{
  ParsersCollection<ROS_Deserializer> parser;
  parser.registerParser("joint_state",
                        GetMessageType<sensor_msgs::JointState>(),
                        GetMessageDefinition<sensor_msgs::JointState>());
  sensor_msgs::JointState joint_state;

  joint_state.header.seq = 2016;
  joint_state.header.stamp.sec = 1234;
  joint_state.header.stamp.nsec = 567 * 1000 * 1000;
  joint_state.header.frame_id = "pippo";

  joint_state.name.resize(3);
  joint_state.position.resize(3);
  joint_state.velocity.resize(3);
  joint_state.effort.resize(3);

  std::string names[3];
  names[0] = ("hola");
  names[1] = ("ciao");
  names[2] = ("bye");

  for (int i = 0; i < 3; i++)
  {
    joint_state.name[i] = names[i];
    joint_state.position[i] = 10 + i;
    joint_state.velocity[i] = 30 + i;
    joint_state.effort[i] = 50 + i;
  }

  std::vector<uint8_t> buffer = BuildMessageBuffer(joint_state);

  std::string* json_txt = parser.deserializeIntoJson("joint_state", Span<uint8_t>(buffer), false);
  // parse the string from format {topic:{t} , msg: {m}} into {m}
  size_t startIdx = json_txt->find("msg");
  if (startIdx != std::string::npos)
  {
    startIdx += 6; // Move past "(msg:)"

    // Find the ending index before the last "}"
    size_t endIdx = json_txt->rfind("}");
    if (endIdx != std::string::npos)
    {
      // Extract the substring
      *json_txt = "{" + json_txt->substr(startIdx, endIdx - startIdx);
    }
    else
    {
      std::cerr << "Error: Unable to find the last '}'." << std::endl;
    }
  }
  else
  {
    std::cerr << "Error: Unable to find '(msg:)'" << std::endl;
  }

  std::vector<uint8_t> bufferOut= parser.serializeFromJson("joint_state", json_txt);

  ros::serialization::IStream sstream(&bufferOut[0], bufferOut.size());
  sensor_msgs::JointState joint_state_from_json;
  ros::serialization::deserialize(sstream, joint_state_from_json);


  auto flat_container = parser.deserialize("joint_state", Span<uint8_t>(bufferOut) );

  CHECK( flat_container->value[0].first.toStdString() == ("joint_state/header/seq"));
  CHECK( flat_container->value[0].second.convert<int>() ==  2016 );

  CHECK( flat_container->value[1].first.toStdString() == ("joint_state/header/stamp"));
  CHECK( flat_container->value[1].second.convert<double>() - 1234.567<=0.01 );
  auto time = flat_container->value[1].second.convert<Time>();
  CHECK( time.sec - joint_state.header.stamp.sec  <=0.001);
  CHECK( time.nsec- joint_state.header.stamp.nsec<=0.001  );

  CHECK( flat_container->value[2].first.toStdString() == ("joint_state/position[0]"));
  CHECK( flat_container->value[2].second.convert<int>() == 10 );
  CHECK( flat_container->value[3].first.toStdString() == ("joint_state/position[1]"));
  CHECK( flat_container->value[3].second.convert<int>() == 11 );
  CHECK( flat_container->value[4].first.toStdString() == ("joint_state/position[2]"));
  CHECK( flat_container->value[4].second.convert<int>() == 12 );

  CHECK( flat_container->value[5].first.toStdString() == ("joint_state/velocity[0]"));
  CHECK( flat_container->value[5].second.convert<int>() == 30 );
  CHECK( flat_container->value[6].first.toStdString() == ("joint_state/velocity[1]"));
  CHECK( flat_container->value[6].second.convert<int>() == 31 );
  CHECK( flat_container->value[7].first.toStdString() == ("joint_state/velocity[2]"));
  CHECK( flat_container->value[7].second.convert<int>() == 32 );

  CHECK( flat_container->value[8].first.toStdString() == ("joint_state/effort[0]"));
  CHECK( flat_container->value[8].second.convert<int>() == 50 );
  CHECK( flat_container->value[9].first.toStdString() == ("joint_state/effort[1]"));
  CHECK( flat_container->value[9].second.convert<int>() == 51 );
  CHECK( flat_container->value[10].first.toStdString() == ("joint_state/effort[2]"));
  CHECK( flat_container->value[10].second.convert<int>() == 52 );


  CHECK( flat_container->name[0].first.toStdString() == ("joint_state/header/frame_id"));
  CHECK( flat_container->name[0].second == ("pippo") );

  CHECK( flat_container->name[1].first.toStdString() == ("joint_state/name[0]"));
  CHECK( flat_container->name[1].second == ("hola") );

  CHECK( flat_container->name[2].first.toStdString() == ("joint_state/name[1]"));
  CHECK( flat_container->name[2].second == ("ciao") );

  CHECK( flat_container->name[3].first.toStdString() == ("joint_state/name[2]"));
  CHECK( flat_container->name[3].second == ("bye") );
}
