#include <gtest/gtest.h>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "rosx_introspection/ros_parser.hpp"
#include "rosx_introspection/ros_utils/ros2_helpers.hpp"

using namespace RosMsgParser;

void print_vector(const char* name, const std::vector<uint8_t>& data)
{
  printf("%s (%d): ", name, (int)data.size());
  for (size_t i = 0; i < data.size(); i++)
  {
    printf("%02X ", data[i]);
  }
  printf("\n");
};

sensor_msgs::msg::JointState BuildSampleJointState()
{
  sensor_msgs::msg::JointState joint_state;

  joint_state.header.stamp.sec = 1234;
  joint_state.header.stamp.nanosec = 567;
  joint_state.header.frame_id = "base";

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

TEST(ParseROS2, JointState)
{
  ParsersCollection<ROS2_Deserializer> parser;
  const std::string topic_type = "sensor_msgs/JointState";

  parser.registerParser("joint_state", ROSType(topic_type),
                        GetMessageDefinition(topic_type));

  auto joint_state = BuildSampleJointState();

  std::vector<uint8_t> buffer = BuildMessageBuffer(joint_state, topic_type);
  print_vector("JointState", buffer);

  auto flat_container = parser.deserialize("joint_state", Span<uint8_t>(buffer));

  for (auto& it : flat_container->value)
  {
    std::cout << it.first << " >> " << it.second.convert<double>() << std::endl;
  }

  for (auto& it : flat_container->name)
  {
    std::cout << it.first << " >> " << it.second << std::endl;
  }

  ASSERT_EQ(flat_container->value[0].first.toStdString(), "joint_state/header/stamp/sec");
  ASSERT_EQ(flat_container->value[0].second.convert<uint32_t>(),
            joint_state.header.stamp.sec);
  ASSERT_EQ(flat_container->value[1].first.toStdString(), "joint_state/header/stamp/"
                                                          "nanosec");
  ASSERT_EQ(flat_container->value[1].second.convert<uint32_t>(),
            joint_state.header.stamp.nanosec);

  ASSERT_EQ(flat_container->value[2].first.toStdString(), ("joint_state/position[0]"));
  ASSERT_EQ(flat_container->value[2].second.convert<int>(), 10);
  ASSERT_EQ(flat_container->value[3].first.toStdString(), ("joint_state/position[1]"));
  ASSERT_EQ(flat_container->value[3].second.convert<int>(), 11);
  ASSERT_EQ(flat_container->value[4].first.toStdString(), ("joint_state/position[2]"));
  ASSERT_EQ(flat_container->value[4].second.convert<int>(), 12);

  ASSERT_EQ(flat_container->value[5].first.toStdString(), ("joint_state/velocity[0]"));
  ASSERT_EQ(flat_container->value[5].second.convert<int>(), 30);
  ASSERT_EQ(flat_container->value[6].first.toStdString(), ("joint_state/velocity[1]"));
  ASSERT_EQ(flat_container->value[6].second.convert<int>(), 31);
  ASSERT_EQ(flat_container->value[7].first.toStdString(), ("joint_state/velocity[2]"));
  ASSERT_EQ(flat_container->value[7].second.convert<int>(), 32);

  ASSERT_EQ(flat_container->value[8].first.toStdString(), ("joint_state/effort[0]"));
  ASSERT_EQ(flat_container->value[8].second.convert<int>(), 50);
  ASSERT_EQ(flat_container->value[9].first.toStdString(), ("joint_state/effort[1]"));
  ASSERT_EQ(flat_container->value[9].second.convert<int>(), 51);
  ASSERT_EQ(flat_container->value[10].first.toStdString(), ("joint_state/effort[2]"));
  ASSERT_EQ(flat_container->value[10].second.convert<int>(), 52);

  ASSERT_EQ(flat_container->name[0].first.toStdString(), ("joint_state/header/frame_id"));
  ASSERT_EQ(flat_container->name[0].second, ("base"));

  ASSERT_EQ(flat_container->name[1].first.toStdString(), ("joint_state/name[0]"));
  ASSERT_EQ(flat_container->name[1].second, ("hola"));

  ASSERT_EQ(flat_container->name[2].first.toStdString(), ("joint_state/name[1]"));
  ASSERT_EQ(flat_container->name[2].second, ("ciao"));

  ASSERT_EQ(flat_container->name[3].first.toStdString(), ("joint_state/name[2]"));
  ASSERT_EQ(flat_container->name[3].second, ("bye"));
}

std::vector<uint8_t> EncodeJointState(const sensor_msgs::msg::JointState& joint_state)
{
  NanoCDR_Serializer encoder;
  encoder.serialize(BuiltinType::INT32, joint_state.header.stamp.sec);
  encoder.serialize(BuiltinType::UINT32, joint_state.header.stamp.nanosec);
  encoder.serializeString(joint_state.header.frame_id);

  encoder.serializeUInt32(joint_state.name.size());
  for (const auto& it : joint_state.name)
  {
    encoder.serializeString(it);
  }
  encoder.serializeUInt32(joint_state.position.size());
  for (const auto& it : joint_state.position)
  {
    encoder.serialize(BuiltinType::FLOAT64, it);
  }
  encoder.serializeUInt32(joint_state.velocity.size());
  for (const auto& it : joint_state.velocity)
  {
    encoder.serialize(BuiltinType::FLOAT64, it);
  }
  encoder.serializeUInt32(joint_state.effort.size());
  for (const auto& it : joint_state.effort)
  {
    encoder.serialize(BuiltinType::FLOAT64, it);
  }
  return std::vector<uint8_t>(encoder.getBufferData(),
                              encoder.getBufferData() + encoder.getBufferSize());
}

void CheckEncoding(const sensor_msgs::msg::JointState& joint_state)
{
  const std::string topic_type = "sensor_msgs/JointState";
  std::vector<uint8_t> ref_buffer = BuildMessageBuffer(joint_state, topic_type);

  auto encoded_buffer = EncodeJointState(joint_state);

  print_vector("Reference", ref_buffer);
  print_vector("Encoded  ", encoded_buffer);

  // Compare the serialized buffer with the reference buffer
  ASSERT_EQ(encoded_buffer.size(), ref_buffer.size());
}

TEST(ParseROS2, JointState_JSON)
{
  const std::string topic_type = "sensor_msgs/JointState";

  Parser parser("joint_state", ROSType(topic_type), GetMessageDefinition(topic_type));
  ROS2_Deserializer deserializer;

  auto joint_state = BuildSampleJointState();

  std::vector<uint8_t> buffer_in = BuildMessageBuffer(joint_state, topic_type);

  std::string json_text;
  parser.deserializeIntoJson(buffer_in, &json_text, &deserializer);

  std::cout << "\n JSON encoding [joint_state]:\n" << json_text << std::endl;

  // test round-robin transform
  ROS2_Serializer serializer;
  parser.serializeFromJson(json_text, &serializer);

  auto joint_state_out = BufferToMessage<sensor_msgs::msg::JointState>(
      serializer.getBufferData(), serializer.getBufferSize());

  ASSERT_EQ(joint_state.header.frame_id, joint_state_out.header.frame_id);
  ASSERT_EQ(joint_state.header.stamp.sec, joint_state_out.header.stamp.sec);
  ASSERT_EQ(joint_state.header.stamp.nanosec, joint_state_out.header.stamp.nanosec);

  for (int i = 0; i < 3; i++)
  {
    ASSERT_EQ(joint_state.name[i], joint_state_out.name[i]);
    ASSERT_EQ(joint_state.position[i], joint_state_out.position[i]);
    ASSERT_EQ(joint_state.velocity[i], joint_state_out.velocity[i]);
    ASSERT_EQ(joint_state.effort[i], joint_state_out.effort[i]);
  }
}

TEST(ParseROS2, JointState_JSON_Omitted)
{
  const char* joint_state_json = R"(
    {"header":{"stamp":{"sec":1234,"nanosec":567000000}},
     "name":["hola","ciao"],
     "position":[10.0,11.0],
     "velocity":[20.0,21.0]
    })";

  const std::string topic_type = "sensor_msgs/JointState";
  Parser parser("joint_state", ROSType(topic_type), GetMessageDefinition(topic_type));

  // We omitted the effort field and the header.frame_id
  ROS2_Serializer serializer;
  parser.serializeFromJson(joint_state_json, &serializer);

  auto joint_state_out = BufferToMessage<sensor_msgs::msg::JointState>(
      serializer.getBufferData(), serializer.getBufferSize());

  ASSERT_EQ("", joint_state_out.header.frame_id);  // default
  ASSERT_EQ(1234, joint_state_out.header.stamp.sec);
  ASSERT_EQ(567000000, joint_state_out.header.stamp.nanosec);

  ASSERT_EQ(2, joint_state_out.name.size());
  ASSERT_EQ(2, joint_state_out.position.size());
  ASSERT_EQ(2, joint_state_out.velocity.size());
  ASSERT_EQ(0, joint_state_out.effort.size());

  ASSERT_EQ("hola", joint_state_out.name[0]);
  ASSERT_EQ(10.0, joint_state_out.position[0]);
  ASSERT_EQ(20.0, joint_state_out.velocity[0]);

  ASSERT_EQ("ciao", joint_state_out.name[1]);
  ASSERT_EQ(11.0, joint_state_out.position[1]);
  ASSERT_EQ(21.0, joint_state_out.velocity[1]);
}

TEST(ParseROS2, PoseStamped_JSON)
{
  const std::string topic_type = "geometry_msgs/PoseStamped";

  Parser parser("joint_state", ROSType(topic_type), GetMessageDefinition(topic_type));
  ROS2_Deserializer deserializer;

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp.sec = 1234;
  pose_stamped.header.stamp.nanosec = 567 * 1000 * 1000;
  pose_stamped.header.frame_id = "base";

  pose_stamped.pose.position.x = 1.0;
  pose_stamped.pose.position.y = 2.0;
  pose_stamped.pose.position.z = 3.0;

  pose_stamped.pose.orientation.x = 0.1;
  pose_stamped.pose.orientation.y = 0.2;
  pose_stamped.pose.orientation.z = 0.3;
  pose_stamped.pose.orientation.w = 0.4;

  std::vector<uint8_t> buffer_in = BuildMessageBuffer(pose_stamped, topic_type);

  std::string json_text;
  parser.deserializeIntoJson(buffer_in, &json_text, &deserializer);

  std::cout << "\n JSON encoding [pose_stamped]:\n" << json_text << std::endl;

  // test round-robin transform
  ROS2_Serializer serializer;
  parser.serializeFromJson(json_text, &serializer);

  auto pose_stamped_out = BufferToMessage<geometry_msgs::msg::PoseStamped>(
      serializer.getBufferData(), serializer.getBufferSize());

  ASSERT_EQ(pose_stamped.header.frame_id, pose_stamped_out.header.frame_id);
  ASSERT_EQ(pose_stamped.header.stamp.sec, pose_stamped_out.header.stamp.sec);
  ASSERT_EQ(pose_stamped.header.stamp.nanosec, pose_stamped_out.header.stamp.nanosec);

  ASSERT_EQ(pose_stamped.pose.position.x, pose_stamped_out.pose.position.x);
  ASSERT_EQ(pose_stamped.pose.position.y, pose_stamped_out.pose.position.y);
  ASSERT_EQ(pose_stamped.pose.position.z, pose_stamped_out.pose.position.z);

  ASSERT_EQ(pose_stamped.pose.orientation.x, pose_stamped_out.pose.orientation.x);
  ASSERT_EQ(pose_stamped.pose.orientation.y, pose_stamped_out.pose.orientation.y);
  ASSERT_EQ(pose_stamped.pose.orientation.z, pose_stamped_out.pose.orientation.z);
  ASSERT_EQ(pose_stamped.pose.orientation.w, pose_stamped_out.pose.orientation.w);
}

TEST(ParseROS2, PoseStamped_JSON_Omitted)
{
  const char* pose_stamped_json = R"(
    {"header":{"stamp":{"sec":1234,"nanosec":567000000},"frame_id":"base"},
     "pose":{"position":{"x":1.0,"y":2.0,"z":3.0}}
     })";

  const std::string topic_type = "geometry_msgs/PoseStamped";
  Parser parser("", ROSType(topic_type), GetMessageDefinition(topic_type));

  // We omitted the effort field and the header.frame_id
  ROS2_Serializer serializer;
  parser.serializeFromJson(pose_stamped_json, &serializer);

  auto pose_stamped_out = BufferToMessage<geometry_msgs::msg::PoseStamped>(
      serializer.getBufferData(), serializer.getBufferSize());

  ASSERT_EQ("base", pose_stamped_out.header.frame_id);
  ASSERT_EQ(1234, pose_stamped_out.header.stamp.sec);
  ASSERT_EQ(567000000, pose_stamped_out.header.stamp.nanosec);

  ASSERT_EQ(1.0, pose_stamped_out.pose.position.x);
  ASSERT_EQ(2.0, pose_stamped_out.pose.position.y);
  ASSERT_EQ(3.0, pose_stamped_out.pose.position.z);

  // Quaternion was omitted, so it should be the default
  ASSERT_EQ(0, pose_stamped_out.pose.orientation.x);
  ASSERT_EQ(0, pose_stamped_out.pose.orientation.y);
  ASSERT_EQ(0, pose_stamped_out.pose.orientation.z);
  ASSERT_EQ(0, pose_stamped_out.pose.orientation.w);
}

TEST(ParseROS2, Duration)
{
  const char* durationA_json = R"({"sec":123,"nanosec":456})";

  const std::string topic_type = "builtin_interfaces/Duration";
  Parser parser("", ROSType(topic_type), GetMessageDefinition(topic_type));

  // We omitted the effort field and the header.frame_id
  ROS2_Serializer serializer;
  parser.serializeFromJson(durationA_json, &serializer);

  auto durationA = BufferToMessage<builtin_interfaces::msg::Duration>(
      serializer.getBufferData(), serializer.getBufferSize());

  ASSERT_EQ(durationA.sec, 123);
  ASSERT_EQ(durationA.nanosec, 456);
  //------------------------------
  const char* durationB_json = R"({"sec":1,"nanosec":234})";
  serializer.reset();
  parser.serializeFromJson(durationB_json, &serializer);

  auto durationB = BufferToMessage<builtin_interfaces::msg::Duration>(
      serializer.getBufferData(), serializer.getBufferSize());

  ASSERT_EQ(durationB.sec, 1);
  ASSERT_EQ(durationB.nanosec, 234);
}
