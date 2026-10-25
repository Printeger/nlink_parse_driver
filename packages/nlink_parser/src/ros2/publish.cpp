#include "publisher.hpp"
#include <cstring>
#define ARRAY_ASSIGN(DEST,SRC) for(size_t k=0;k<sizeof(SRC)/sizeof(SRC[0]);++k) DEST[k]=SRC[k];
namespace nlink {
Publisher::Publisher(rclcpp::Node& node,const rclcpp::QoS& qos) {
pub_anchorframe0=node.create_publisher<nlink_parser::msg::LinktrackAnchorframe0>("nlink_linktrack_anchorframe0",qos);
pub_tagframe0=node.create_publisher<nlink_parser::msg::LinktrackTagframe0>("nlink_linktrack_tagframe0",qos);
pub_nodeframe0=node.create_publisher<nlink_parser::msg::LinktrackNodeframe0>("nlink_linktrack_nodeframe0",qos);
stamped_nodeframe0=node.create_publisher<nlink_parser::msg::LinktrackNodeframe0Stamped>("nlink_linktrack_nodeframe0_stamped",qos);
pub_nodeframe1=node.create_publisher<nlink_parser::msg::LinktrackNodeframe1>("nlink_linktrack_nodeframe1",qos);
pub_nodeframe2=node.create_publisher<nlink_parser::msg::LinktrackNodeframe2>("nlink_linktrack_nodeframe2",qos);
stamped_nodeframe2=node.create_publisher<nlink_parser::msg::LinktrackNodeframe2Stamped>("nlink_linktrack_nodeframe2_stamped",qos);
pub_nodeframe3=node.create_publisher<nlink_parser::msg::LinktrackNodeframe3>("nlink_linktrack_nodeframe3",qos);
stamped_nodeframe3=node.create_publisher<nlink_parser::msg::LinktrackNodeframe3Stamped>("nlink_linktrack_nodeframe3_stamped",qos);
pub_nodeframe4=node.create_publisher<nlink_parser::msg::LinktrackNodeframe4>("nlink_linktrack_nodeframe4",qos);
pub_nodeframe5=node.create_publisher<nlink_parser::msg::LinktrackNodeframe5>("nlink_linktrack_nodeframe5",qos);
stamped_nodeframe5=node.create_publisher<nlink_parser::msg::LinktrackNodeframe5Stamped>("nlink_linktrack_nodeframe5_stamped",qos);
pub_nodeframe6=node.create_publisher<nlink_parser::msg::LinktrackNodeframe6>("nlink_linktrack_nodeframe6",qos);
stamped_nodeframe6=node.create_publisher<nlink_parser::msg::LinktrackNodeframe6Stamped>("nlink_linktrack_nodeframe6_stamped",qos);
}
bool Publisher::publish(const uint8_t* bytes,size_t length,const nlink_parser::msg::NLinkRxContext& context) { switch(bytes[1]) {
case 0: {
 if(!nlt_anchorframe0_.UnpackData(bytes,length)) return false;
 nlink_parser::msg::LinktrackAnchorframe0 msg_data;
    auto data = nlt_anchorframe0_.result;
    msg_data.role = data.role;
    msg_data.id = data.id;
    msg_data.voltage = data.voltage;
    msg_data.local_time = data.local_time;
    msg_data.system_time = data.system_time;
    auto &msg_nodes = msg_data.nodes;
    msg_nodes.clear();
    decltype(msg_data.nodes)::value_type msg_node;
    for (size_t i = 0, icount = data.valid_node_count; i < icount; ++i) {
      auto node = data.nodes[i];
      msg_node.role = node->role;
      msg_node.id = node->id;
      ARRAY_ASSIGN(msg_node.pos_3d, node->pos_3d)
      ARRAY_ASSIGN(msg_node.dis_arr, node->dis_arr)
      msg_nodes.push_back(msg_node);
    }
 pub_anchorframe0->publish(msg_data);
 return true; }
case 1: {
 if(!g_nlt_tagframe0.UnpackData(bytes,length)) return false;
 nlink_parser::msg::LinktrackTagframe0 msg_data;
    const auto &data = g_nlt_tagframe0.result;


    msg_data.role = data.role;
    msg_data.id = data.id;
    msg_data.local_time = data.local_time;
    msg_data.system_time = data.system_time;
    msg_data.voltage = data.voltage;
    ARRAY_ASSIGN(msg_data.pos_3d, data.pos_3d)
    ARRAY_ASSIGN(msg_data.eop_3d, data.eop_3d)
    ARRAY_ASSIGN(msg_data.vel_3d, data.vel_3d)
    ARRAY_ASSIGN(msg_data.dis_arr, data.dis_arr)
    ARRAY_ASSIGN(msg_data.imu_gyro_3d, data.imu_gyro_3d)
    ARRAY_ASSIGN(msg_data.imu_acc_3d, data.imu_acc_3d)
    ARRAY_ASSIGN(msg_data.angle_3d, data.angle_3d)
    ARRAY_ASSIGN(msg_data.quaternion, data.quaternion)

 pub_tagframe0->publish(msg_data);
 return true; }
case 2: {
 if(!g_nlt_nodeframe0.UnpackData(bytes,length)) return false;
 nlink_parser::msg::LinktrackNodeframe0 msg_data;
    const auto &data = g_nlt_nodeframe0.result;

    auto &msg_nodes = msg_data.nodes;

    msg_data.role = data.role;
    msg_data.id = data.id;

    msg_nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg_nodes[i];
      auto node = data.nodes[i];
      msg_node.id = node->id;
      msg_node.role = node->role;
      msg_node.data.resize(node->data_length);
      memcpy(msg_node.data.data(), node->data, node->data_length);
    }

 pub_nodeframe0->publish(msg_data);
 nlink_parser::msg::LinktrackNodeframe0Stamped stamped; stamped.context=context; stamped.frame=msg_data; stamped_nodeframe0->publish(stamped);
 return true; }
case 3: {
 if(!g_nlt_nodeframe1.UnpackData(bytes,length)) return false;
 nlink_parser::msg::LinktrackNodeframe1 msg_data;
    const auto &data = g_nlt_nodeframe1.result;

    auto &msg_nodes = msg_data.nodes;

    msg_data.local_time = data.local_time;

    msg_data.role = data.role;
    msg_data.id = data.id;
    msg_data.system_time = data.system_time;
    msg_data.voltage = data.voltage;

    msg_nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg_nodes[i];
      auto node = data.nodes[i];
      msg_node.id = node->id;
      msg_node.role = node->role;
      ARRAY_ASSIGN(msg_node.pos_3d, node->pos_3d)
    }

 pub_nodeframe1->publish(msg_data);
 return true; }
case 4: {
 if(!g_nlt_nodeframe2.UnpackData(bytes,length)) return false;
 nlink_parser::msg::LinktrackNodeframe2 msg_data;
    const auto &data = g_nlt_nodeframe2.result;

    auto &msg_nodes = msg_data.nodes;


    msg_data.role = data.role;
    msg_data.id = data.id;
    msg_data.local_time = data.local_time;
    msg_data.system_time = data.system_time;
    msg_data.voltage = data.voltage;
    ARRAY_ASSIGN(msg_data.pos_3d, data.pos_3d)
    ARRAY_ASSIGN(msg_data.eop_3d, data.eop_3d)
    ARRAY_ASSIGN(msg_data.vel_3d, data.vel_3d)
    ARRAY_ASSIGN(msg_data.imu_gyro_3d, data.imu_gyro_3d)
    ARRAY_ASSIGN(msg_data.imu_acc_3d, data.imu_acc_3d)
    ARRAY_ASSIGN(msg_data.angle_3d, data.angle_3d)
    ARRAY_ASSIGN(msg_data.quaternion, data.quaternion)

    msg_nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg_nodes[i];
      auto node = data.nodes[i];
      msg_node.id = node->id;
      msg_node.role = node->role;
      msg_node.dis = node->dis;
      msg_node.fp_rssi = node->fp_rssi;
      msg_node.rx_rssi = node->rx_rssi;
    }

 pub_nodeframe2->publish(msg_data);
 nlink_parser::msg::LinktrackNodeframe2Stamped stamped; stamped.context=context; stamped.frame=msg_data; stamped_nodeframe2->publish(stamped);
 return true; }
case 5: {
 if(!g_nlt_nodeframe3.UnpackData(bytes,length)) return false;
 nlink_parser::msg::LinktrackNodeframe3 msg_data;
    const auto &data = g_nlt_nodeframe3.result;

    auto &msg_nodes = msg_data.nodes;

    
    msg_data.header.stamp = context.header.stamp;
    msg_data.role = data.role;
    msg_data.id = data.id;
    msg_data.local_time = data.local_time;
    msg_data.system_time = data.system_time;
    msg_data.voltage = data.voltage;

    msg_nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg_nodes[i];
      auto node = data.nodes[i];
      msg_node.id = node->id;
      msg_node.role = node->role;
      msg_node.dis = node->dis;
      msg_node.fp_rssi = node->fp_rssi;
      msg_node.rx_rssi = node->rx_rssi;
    }

 pub_nodeframe3->publish(msg_data);
 nlink_parser::msg::LinktrackNodeframe3Stamped stamped; stamped.context=context; stamped.frame=msg_data; stamped_nodeframe3->publish(stamped);
 return true; }
case 6: {
 if(!g_nlt_nodeframe4.UnpackData(bytes,length)) return false;
 nlink_parser::msg::LinktrackNodeframe4 msg_data;
    const auto &data = g_nlt_nodeframe4.result;

    msg_data.role = data.role;
    msg_data.id = data.id;
    msg_data.local_time = data.local_time;
    msg_data.system_time = data.system_time;
    msg_data.voltage = data.voltage;
    msg_data.tags.resize(data.tag_count);
    for (int i = 0; i < data.tag_count; ++i) {
      auto &msg_tag = msg_data.tags[i];
      auto tag = data.tags[i];
      msg_tag.id = tag->id;
      msg_tag.voltage = tag->voltage;
      msg_tag.anchors.resize(tag->anchor_count);
      for (int j = 0; j < tag->anchor_count; ++j) {
        auto &msg_anchor = msg_tag.anchors[j];
        auto anchor = tag->anchors[j];
        msg_anchor.id = anchor->id;
        msg_anchor.dis = anchor->dis;
      }
    }

 pub_nodeframe4->publish(msg_data);
 return true; }
case 8: {
 if(!g_nlt_nodeframe5.UnpackData(bytes,length)) return false;
 nlink_parser::msg::LinktrackNodeframe5 msg_data;
    const auto &data = g_nlt_nodeframe5.result;

    auto &msg_nodes = msg_data.nodes;

    msg_data.local_time = data.local_time;

    msg_data.role = data.role;
    msg_data.id = data.id;
    msg_data.system_time = data.system_time;
    msg_data.voltage = data.voltage;

    msg_nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg_nodes[i];
      auto node = data.nodes[i];
      msg_node.id = node->id;
      msg_node.role = node->role;
      msg_node.dis = node->dis;
      msg_node.fp_rssi = node->fp_rssi;
      msg_node.rx_rssi = node->rx_rssi;
    }

 pub_nodeframe5->publish(msg_data);
 nlink_parser::msg::LinktrackNodeframe5Stamped stamped; stamped.context=context; stamped.frame=msg_data; stamped_nodeframe5->publish(stamped);
 return true; }
case 9: {
 if(!g_nlt_nodeframe6.UnpackData(bytes,length)) return false;
 nlink_parser::msg::LinktrackNodeframe6 msg_data;
    const auto &data = g_nlt_nodeframe6.result;

    auto &msg_nodes = msg_data.nodes;

    msg_data.role = data.role;
    msg_data.id = data.id;

    msg_nodes.resize(data.valid_node_count);
    for (size_t i = 0; i < data.valid_node_count; ++i) {
      auto &msg_node = msg_nodes[i];
      auto node = data.nodes[i];
      msg_node.id = node->id;
      msg_node.role = node->role;
      msg_node.data.resize(node->data_length);
      memcpy(msg_node.data.data(), node->data, node->data_length);
    }

 pub_nodeframe6->publish(msg_data);
 nlink_parser::msg::LinktrackNodeframe6Stamped stamped; stamped.context=context; stamped.frame=msg_data; stamped_nodeframe6->publish(stamped);
 return true; }
default:return false; } }
}
