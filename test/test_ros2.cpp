#include "doctest.h"

#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "rosx_introspection/ros_parser.hpp"
#include "rosx_introspection/ros_utils/ros2_helpers.hpp"

using namespace RosMsgParser;

sensor_msgs::msg::JointState BuildSampleJointState()
{
  sensor_msgs::msg::JointState joint_state;

  joint_state.header.stamp.sec = 1234;
  joint_state.header.stamp.nanosec = 567 * 1000 * 1000;
  joint_state.header.frame_id = "pippo";

  joint_state.name.resize(3);
  joint_state.position.resize(3);
  joint_state.velocity.resize(3);
  joint_state.effort.resize(3);

  joint_state.name[0] = "hola";
  joint_state.name[1] = "ciao";
  joint_state.name[2] = "bye";

  for (int i = 0; i < 3; i++)
  {
    joint_state.position[i] = 10 + i;
    joint_state.velocity[i] = 30 + i;
    joint_state.effort[i] = 50 + i;
  }
  return joint_state;
}

TEST_CASE("Parse ROS2 [JointState]")
{
  ParsersCollection<ROS2_Deserializer> parser;
  const std::string topic_type = "sensor_msgs/JointState";

  parser.registerParser("joint_state", ROSType(topic_type),
                        GetMessageDefinition(topic_type));

  auto joint_state = BuildSampleJointState();

  std::vector<uint8_t> buffer = BuildMessageBuffer(joint_state, topic_type);

  auto flat_container = parser.deserialize("joint_state", Span<uint8_t>(buffer));

  for (auto& it : flat_container->value)
  {
    std::cout << it.first << " >> " << it.second.convert<double>() << std::endl;
  }

  for (auto& it : flat_container->name)
  {
    std::cout << it.first << " >> " << it.second << std::endl;
  }

  CHECK(flat_container->value[0].first.toStdString() == ("joint_state/header/stamp/sec"));
  CHECK(flat_container->value[0].second.convert<uint32_t>() ==
        joint_state.header.stamp.sec);
  CHECK(flat_container->value[1].first.toStdString() == ("joint_state/header/stamp/"
                                                         "nanosec"));
  CHECK(flat_container->value[1].second.convert<uint32_t>() ==
        joint_state.header.stamp.nanosec);

  CHECK(flat_container->value[2].first.toStdString() == ("joint_state/position[0]"));
  CHECK(flat_container->value[2].second.convert<int>() == 10);
  CHECK(flat_container->value[3].first.toStdString() == ("joint_state/position[1]"));
  CHECK(flat_container->value[3].second.convert<int>() == 11);
  CHECK(flat_container->value[4].first.toStdString() == ("joint_state/position[2]"));
  CHECK(flat_container->value[4].second.convert<int>() == 12);

  CHECK(flat_container->value[5].first.toStdString() == ("joint_state/velocity[0]"));
  CHECK(flat_container->value[5].second.convert<int>() == 30);
  CHECK(flat_container->value[6].first.toStdString() == ("joint_state/velocity[1]"));
  CHECK(flat_container->value[6].second.convert<int>() == 31);
  CHECK(flat_container->value[7].first.toStdString() == ("joint_state/velocity[2]"));
  CHECK(flat_container->value[7].second.convert<int>() == 32);

  CHECK(flat_container->value[8].first.toStdString() == ("joint_state/effort[0]"));
  CHECK(flat_container->value[8].second.convert<int>() == 50);
  CHECK(flat_container->value[9].first.toStdString() == ("joint_state/effort[1]"));
  CHECK(flat_container->value[9].second.convert<int>() == 51);
  CHECK(flat_container->value[10].first.toStdString() == ("joint_state/effort[2]"));
  CHECK(flat_container->value[10].second.convert<int>() == 52);

  CHECK(flat_container->name[0].first.toStdString() == ("joint_state/header/frame_id"));
  CHECK(flat_container->name[0].second == ("pippo"));

  CHECK(flat_container->name[1].first.toStdString() == ("joint_state/name[0]"));
  CHECK(flat_container->name[1].second == ("hola"));

  CHECK(flat_container->name[2].first.toStdString() == ("joint_state/name[1]"));
  CHECK(flat_container->name[2].second == ("ciao"));

  CHECK(flat_container->name[3].first.toStdString() == ("joint_state/name[2]"));
  CHECK(flat_container->name[3].second == ("bye"));
}

TEST_CASE("Parse ROS2 [JointState_JSON]")
{
  const std::string topic_type = "sensor_msgs/JointState";

  Parser parser("joint_state", ROSType(topic_type), GetMessageDefinition(topic_type));
  ROS2_Deserializer deserializer;

  auto joint_state = BuildSampleJointState();

  std::vector<uint8_t> buffer_in = BuildMessageBuffer(joint_state, topic_type);

  std::string json_text;
  parser.deserializeIntoJson(buffer_in, &json_text, &deserializer);

  std::cout << "\n JSON encoding:\n" << json_text << std::endl;

  // test round-robin transform
  ROS2_Serializer serializer;
  parser.serializeFromJson(json_text, &serializer);

  std::vector<uint8_t> buffer_out(serializer.getBufferSize());
  memcpy(buffer_out.data(), serializer.getBufferData(), buffer_out.size());

  auto joint_state_out = BufferToMessage<sensor_msgs::msg::JointState>(buffer_out);

  CHECK(buffer_in.size() == buffer_out.size());

  CHECK(joint_state.header.frame_id == joint_state_out.header.frame_id);
  CHECK(joint_state.header.stamp.sec == joint_state_out.header.stamp.sec);
  CHECK(joint_state.header.stamp.nanosec == joint_state_out.header.stamp.nanosec);

  for (int i = 0; i < 3; i++)
  {
    CHECK(joint_state.name[i] == joint_state_out.name[i]);
    CHECK(joint_state.position[i] == joint_state_out.position[i]);
    CHECK(joint_state.velocity[i] == joint_state_out.velocity[i]);
    CHECK(joint_state.effort[i] == joint_state_out.effort[i]);
  }
}
