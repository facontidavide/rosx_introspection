#include <gtest/gtest.h>

#include "rosx_introspection/ros_message.hpp"
#include "rosx_introspection/stringtree_leaf.hpp"

const std::string vector_def = "# This represents a vector in free space. \n"
                               "# It is only meant to represent a direction. Therefore, "
                               "it does not\n"
                               "# make sense to apply a translation to it (e.g., when "
                               "applying a \n"
                               "# generic rigid transformation to a Vector3, tf2 will "
                               "only apply the\n"
                               "# rotation). If you want your data to be translatable "
                               "too, use the\n"
                               "# geometry_msgs/Point message instead.\n"
                               "\n"
                               "float64 x\n"
                               "float64 y\n"
                               "float64 z\n";

using namespace RosMsgParser;

TEST(Parser, BasicTest)
{
  ROSMessage msg(vector_def);

  ASSERT_EQ(msg.fields().size(), 3);

  ASSERT_EQ(msg.field(0).name(), "x");
  ASSERT_EQ(msg.field(1).name(), "y");
  ASSERT_EQ(msg.field(2).name(), "z");

  ASSERT_EQ(msg.field(0).type().typeID(), FLOAT64);
  ASSERT_EQ(msg.field(1).type().typeID(), FLOAT64);
  ASSERT_EQ(msg.field(2).type().typeID(), FLOAT64);
}

const std::string pose_stamped_def = "# A Pose with reference coordinate frame and "
                                     "timestamp\n"
                                     "Header header\n"
                                     "Pose pose\n"
                                     "\n"
                                     "==================================================="
                                     "=============================\n"
                                     "MSG: std_msgs/Header\n"
                                     "# Standard metadata for higher-level stamped data "
                                     "types.\n"
                                     "# This is generally used to communicate "
                                     "timestamped data \n"
                                     "# in a particular coordinate frame.\n"
                                     "# \n"
                                     "# sequence ID: consecutively increasing ID \n"
                                     "uint32 seq\n"
                                     "#Two-integer timestamp that is expressed as:\n"
                                     "# * stamp.sec: seconds (stamp_secs) since epoch "
                                     "(in Python the variable is called 'secs')\n"
                                     "# * stamp.nsec: nanoseconds since stamp_secs (in "
                                     "Python the variable is called 'nsecs')\n"
                                     "# time-handling sugar is provided by the client "
                                     "library\n"
                                     "time stamp\n"
                                     "#Frame this data is associated with\n"
                                     "string frame_id\n"
                                     "\n"
                                     "==================================================="
                                     "=============================\n"
                                     "MSG: geometry_msgs/Pose\n"
                                     "# A representation of pose in free space, composed "
                                     "of position and orientation. \n"
                                     "Point position\n"
                                     "Quaternion orientation\n"
                                     "\n"
                                     "==================================================="
                                     "=============================\n"
                                     "MSG: geometry_msgs/Point\n"
                                     "# This contains the position of a point in free "
                                     "space\n"
                                     "float64 x\n"
                                     "float64 y\n"
                                     "float64 z\n"
                                     "\n"
                                     "==================================================="
                                     "=============================\n"
                                     "MSG: geometry_msgs/Quaternion\n"
                                     "# This represents an orientation in free space in "
                                     "quaternion form.\n"
                                     "\n"
                                     "float64 x\n"
                                     "float64 y\n"
                                     "float64 z\n"
                                     "float64 w\n";

TEST(Parser, CompositeROS1)
{
  auto msg_parsed = ParseMessageDefinitions(pose_stamped_def, ROSType("geometry_msgs/"
                                                                      "PoseStamped"));

  ASSERT_EQ(msg_parsed.size(), 5);

  auto pose_stamped = msg_parsed[0];
  auto header = msg_parsed[1];
  auto pose = msg_parsed[2];
  auto point = msg_parsed[3];
  auto quaternion = msg_parsed[4];

  ASSERT_EQ(pose_stamped->type().baseName(), "geometry_msgs/PoseStamped");
  ASSERT_EQ(pose_stamped->fields().size(), 2);
  ASSERT_EQ(pose_stamped->field(0).type().baseName(), "std_msgs/Header");
  ASSERT_EQ(pose_stamped->field(1).type().baseName(), "geometry_msgs/Pose");

  ASSERT_EQ(header->type().baseName(), "std_msgs/Header");
  ASSERT_EQ(header->fields().size(), 3);
  ASSERT_EQ(header->field(0).type().baseName(), "uint32");
  ASSERT_EQ(header->field(1).type().baseName(), "time");
  ASSERT_EQ(header->field(2).type().baseName(), "string");

  ASSERT_EQ(pose->type().baseName(), "geometry_msgs/Pose");
  ASSERT_EQ(pose->fields().size(), 2);
  ASSERT_EQ(pose->field(0).type().baseName(), "geometry_msgs/Point");
  ASSERT_EQ(pose->field(1).type().baseName(), "geometry_msgs/Quaternion");

  ASSERT_EQ(point->type().baseName(), "geometry_msgs/Point");
  ASSERT_EQ(point->fields().size(), 3);
  ASSERT_EQ(point->field(0).type().baseName(), "float64");
  ASSERT_EQ(point->field(1).type().baseName(), "float64");
  ASSERT_EQ(point->field(2).type().baseName(), "float64");

  ASSERT_EQ(quaternion->type().baseName(), "geometry_msgs/Quaternion");
  ASSERT_EQ(quaternion->fields().size(), 4);
  ASSERT_EQ(quaternion->field(0).type().baseName(), "float64");
  ASSERT_EQ(quaternion->field(1).type().baseName(), "float64");
  ASSERT_EQ(quaternion->field(2).type().baseName(), "float64");
  ASSERT_EQ(quaternion->field(3).type().baseName(), "float64");

  //--------------------------------------
  MessageSchema::Ptr schema = BuildMessageSchema("pose_stamped", msg_parsed);

  ASSERT_EQ(schema->field_tree.root()->children().size(), 2);

  std::vector<std::string> leaf_str;

  auto recursiveLeaf = std::function<void(const FieldTreeNode* node)>();
  recursiveLeaf = [&](const FieldTreeNode* node) {
    if (node->isLeaf())
    {
      FieldLeaf leaf;
      leaf.node = node;
      FieldsVector fields_vector(leaf);
      leaf_str.push_back(fields_vector.toStdString());
    }
    else
    {
      for (const auto& child_node : node->children())
      {
        recursiveLeaf(&child_node);
      }
    }
  };
  recursiveLeaf(schema->field_tree.root());

  ASSERT_EQ(leaf_str.size(), 10);

  size_t index = 0;
  ASSERT_EQ(leaf_str[index++], "pose_stamped/header/seq");
  ASSERT_EQ(leaf_str[index++], "pose_stamped/header/stamp");
  ASSERT_EQ(leaf_str[index++], "pose_stamped/header/frame_id");

  ASSERT_EQ(leaf_str[index++], "pose_stamped/pose/position/x");
  ASSERT_EQ(leaf_str[index++], "pose_stamped/pose/position/y");
  ASSERT_EQ(leaf_str[index++], "pose_stamped/pose/position/z");

  ASSERT_EQ(leaf_str[index++], "pose_stamped/pose/orientation/x");
  ASSERT_EQ(leaf_str[index++], "pose_stamped/pose/orientation/y");
  ASSERT_EQ(leaf_str[index++], "pose_stamped/pose/orientation/z");
  ASSERT_EQ(leaf_str[index++], "pose_stamped/pose/orientation/w");
}

TEST(Parser, QuaternionFieldROS2)
{
  ROSField field("float64 x 0");

  ASSERT_EQ(field.type().typeID(), FLOAT64);
  ASSERT_EQ(field.name(), "x");
}
