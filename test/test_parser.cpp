#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"
#include "rosx_introspection/ros_message.hpp"
#include "rosx_introspection/stringtree_leaf.hpp"

const std::string vector_def =
  "# This represents a vector in free space. \n"
  "# It is only meant to represent a direction. Therefore, it does not\n"
  "# make sense to apply a translation to it (e.g., when applying a \n"
  "# generic rigid transformation to a Vector3, tf2 will only apply the\n"
  "# rotation). If you want your data to be translatable too, use the\n"
  "# geometry_msgs/Point message instead.\n"
  "\n"
  "float64 x\n"
  "float64 y\n"
  "float64 z\n";

using namespace RosMsgParser;

TEST_CASE("Parser basic test") {

  ROSMessage msg(vector_def);

  CHECK(msg.fields().size() == 3);

  CHECK(msg.field(0).name() == "x");
  CHECK(msg.field(1).name() == "y");
  CHECK(msg.field(2).name() == "z");

  CHECK(msg.field(0).type().typeID() == FLOAT64);
  CHECK(msg.field(1).type().typeID() == FLOAT64);
  CHECK(msg.field(2).type().typeID() == FLOAT64);
}

const std::string pose_stamped_def =
  "# A Pose with reference coordinate frame and timestamp\n"
  "Header header\n"
  "Pose pose\n"
  "\n"
  "================================================================================\n"
  "MSG: std_msgs/Header\n"
  "# Standard metadata for higher-level stamped data types.\n"
  "# This is generally used to communicate timestamped data \n"
  "# in a particular coordinate frame.\n"
  "# \n"
  "# sequence ID: consecutively increasing ID \n"
  "uint32 seq\n"
  "#Two-integer timestamp that is expressed as:\n"
  "# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
  "# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
  "# time-handling sugar is provided by the client library\n"
  "time stamp\n"
  "#Frame this data is associated with\n"
  "string frame_id\n"
  "\n"
  "================================================================================\n"
  "MSG: geometry_msgs/Pose\n"
  "# A representation of pose in free space, composed of position and orientation. \n"
  "Point position\n"
  "Quaternion orientation\n"
  "\n"
  "================================================================================\n"
  "MSG: geometry_msgs/Point\n"
  "# This contains the position of a point in free space\n"
  "float64 x\n"
  "float64 y\n"
  "float64 z\n"
  "\n"
  "================================================================================\n"
  "MSG: geometry_msgs/Quaternion\n"
  "# This represents an orientation in free space in quaternion form.\n"
  "\n"
  "float64 x\n"
  "float64 y\n"
  "float64 z\n"
  "float64 w\n"
  ;

TEST_CASE("Parser Composite ROS1") {

  auto msg_parsed=  ParseMessageDefinitions(pose_stamped_def,
                                            ROSType("geometry_msgs/PoseStamped"));

  CHECK(msg_parsed.size() == 5);

  auto pose_stamped = msg_parsed[0];
  auto header = msg_parsed[1];
  auto pose = msg_parsed[2];
  auto point = msg_parsed[3];
  auto quaternion = msg_parsed[4];

  CHECK(pose_stamped->type().baseName() == "geometry_msgs/PoseStamped");
  CHECK(pose_stamped->fields().size() == 2);
  CHECK(pose_stamped->field(0).type().baseName() == "std_msgs/Header");
  CHECK(pose_stamped->field(1).type().baseName() == "geometry_msgs/Pose");

  CHECK(header->type().baseName() == "std_msgs/Header");
  CHECK(header->fields().size() == 3);
  CHECK(header->field(0).type().baseName() == "uint32");
  CHECK(header->field(1).type().baseName() == "time");
  CHECK(header->field(2).type().baseName() == "string");

  CHECK(pose->type().baseName() == "geometry_msgs/Pose");
  CHECK(pose->fields().size() == 2);
  CHECK(pose->field(0).type().baseName() == "geometry_msgs/Point");
  CHECK(pose->field(1).type().baseName() == "geometry_msgs/Quaternion");

  CHECK(point->type().baseName() == "geometry_msgs/Point");
  CHECK(point->fields().size() == 3);
  CHECK(point->field(0).type().baseName() == "float64");
  CHECK(point->field(1).type().baseName() == "float64");
  CHECK(point->field(2).type().baseName() == "float64");

  CHECK(quaternion->type().baseName() == "geometry_msgs/Quaternion");
  CHECK(quaternion->fields().size() == 4);
  CHECK(quaternion->field(0).type().baseName() == "float64");
  CHECK(quaternion->field(1).type().baseName() == "float64");
  CHECK(quaternion->field(2).type().baseName() == "float64");
  CHECK(quaternion->field(3).type().baseName() == "float64");

  //--------------------------------------
  MessageSchema::Ptr schema = BuildMessageSchema("pose_stamped", msg_parsed);

  CHECK( schema->field_tree.root()->children().size() == 2 );

  std::vector<std::string> leaf_str;

  auto recursiveLeaf = std::function<void(const FieldTreeNode* node)>();
  recursiveLeaf = [&](const FieldTreeNode* node)
  {
    if(node->isLeaf())
    {
      FieldLeaf leaf;
      leaf.node = node;
      FieldsVector fields_vector( leaf );
      leaf_str.push_back( fields_vector.toStdString() );
    }
    else{
      for(const auto& child_node: node->children())
      {
        recursiveLeaf(&child_node);
      }
    }
  };
  recursiveLeaf( schema->field_tree.root() );

  CHECK( leaf_str.size() == 10);

  size_t index = 0;
  CHECK( leaf_str[index++] == "pose_stamped/header/seq");
  CHECK( leaf_str[index++] == "pose_stamped/header/stamp");
  CHECK( leaf_str[index++] == "pose_stamped/header/frame_id");

  CHECK( leaf_str[index++] == "pose_stamped/pose/position/x");
  CHECK( leaf_str[index++] == "pose_stamped/pose/position/y");
  CHECK( leaf_str[index++] == "pose_stamped/pose/position/z");

  CHECK( leaf_str[index++] == "pose_stamped/pose/orientation/x");
  CHECK( leaf_str[index++] == "pose_stamped/pose/orientation/y");
  CHECK( leaf_str[index++] == "pose_stamped/pose/orientation/z");
  CHECK( leaf_str[index++] == "pose_stamped/pose/orientation/w");
}

