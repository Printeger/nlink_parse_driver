#include "publisher.hpp"
#include <cstring>
#include <stdexcept>
#define ARRAY_ASSIGN(DEST,SRC) for(size_t k=0;k<sizeof(SRC)/sizeof(SRC[0]);++k) DEST[k]=SRC[k];
namespace nlink {
namespace {
template<class T> T parameter(ros::NodeHandle& node,const std::string& key,const T& fallback) {
  if(!node.hasParam(key)) return fallback;
  T value;
  if(!node.getParam(key,value)) throw std::invalid_argument("invalid parameter type: ~"+key);
  return value;
}
}

Publisher::Publisher(ros::NodeHandle& node,ros::NodeHandle& private_node) {
  const int range=parameter<int>(private_node,"range_frame",3);
  const int data=parameter<int>(private_node,"data_frame",0);
  const bool debug=parameter<bool>(private_node,"publish_debug_topics",false);
  if(range!=2 && range!=3 && range!=5)
    throw std::invalid_argument("range_frame must be 2/3/5");
  if(data!=0 && data!=6)
    throw std::invalid_argument("data_frame must be 0/6");

  // Raw outputs and unselected stamped streams are opt-in diagnostics.
  if(debug) {
    pub_anchorframe0=node.advertise<nlink_parser::LinktrackAnchorframe0>("nlink_linktrack_anchorframe0",100,false);
    pub_tagframe0=node.advertise<nlink_parser::LinktrackTagframe0>("nlink_linktrack_tagframe0",100,false);
    pub_nodeframe0=node.advertise<nlink_parser::LinktrackNodeframe0>("nlink_linktrack_nodeframe0",100,false);
    pub_nodeframe1=node.advertise<nlink_parser::LinktrackNodeframe1>("nlink_linktrack_nodeframe1",100,false);
    pub_nodeframe2=node.advertise<nlink_parser::LinktrackNodeframe2>("nlink_linktrack_nodeframe2",100,false);
    pub_nodeframe3=node.advertise<nlink_parser::LinktrackNodeframe3>("nlink_linktrack_nodeframe3",100,false);
    pub_nodeframe4=node.advertise<nlink_parser::LinktrackNodeframe4>("nlink_linktrack_nodeframe4",100,false);
    pub_nodeframe5=node.advertise<nlink_parser::LinktrackNodeframe5>("nlink_linktrack_nodeframe5",100,false);
    pub_nodeframe6=node.advertise<nlink_parser::LinktrackNodeframe6>("nlink_linktrack_nodeframe6",100,false);
  }
  if(debug || data==0)
    stamped_nodeframe0=node.advertise<nlink_parser::LinktrackNodeframe0Stamped>("nlink_linktrack_nodeframe0_stamped",100,false);
  if(debug || range==2)
    stamped_nodeframe2=node.advertise<nlink_parser::LinktrackNodeframe2Stamped>("nlink_linktrack_nodeframe2_stamped",100,false);
  if(debug || range==3)
    stamped_nodeframe3=node.advertise<nlink_parser::LinktrackNodeframe3Stamped>("nlink_linktrack_nodeframe3_stamped",100,false);
  if(debug || range==5)
    stamped_nodeframe5=node.advertise<nlink_parser::LinktrackNodeframe5Stamped>("nlink_linktrack_nodeframe5_stamped",100,false);
  if(debug || data==6)
    stamped_nodeframe6=node.advertise<nlink_parser::LinktrackNodeframe6Stamped>("nlink_linktrack_nodeframe6_stamped",100,false);
}
bool Publisher::publish(const uint8_t* bytes,size_t length,const nlink_parser::NLinkRxContext& context) { switch(bytes[1]) {
case 0: {
 if(!nlt_anchorframe0_.UnpackData(bytes,length)) return false;
 if(!(pub_anchorframe0)) return true;
 nlink_parser::LinktrackAnchorframe0 msg_data;
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
 if(pub_anchorframe0) pub_anchorframe0.publish(msg_data);
 return true; }
case 1: {
 if(!g_nlt_tagframe0.UnpackData(bytes,length)) return false;
 if(!(pub_tagframe0)) return true;
 nlink_parser::LinktrackTagframe0 msg_data;
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

 if(pub_tagframe0) pub_tagframe0.publish(msg_data);
 return true; }
case 2: {
 if(!g_nlt_nodeframe0.UnpackData(bytes,length)) return false;
 if(!(pub_nodeframe0 || stamped_nodeframe0)) return true;
 nlink_parser::LinktrackNodeframe0 msg_data;
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

 if(pub_nodeframe0) pub_nodeframe0.publish(msg_data);
 if(stamped_nodeframe0) { nlink_parser::LinktrackNodeframe0Stamped stamped; stamped.context=context; stamped.frame=msg_data; stamped_nodeframe0.publish(stamped); }
 return true; }
case 3: {
 if(!g_nlt_nodeframe1.UnpackData(bytes,length)) return false;
 if(!(pub_nodeframe1)) return true;
 nlink_parser::LinktrackNodeframe1 msg_data;
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

 if(pub_nodeframe1) pub_nodeframe1.publish(msg_data);
 return true; }
case 4: {
 if(!g_nlt_nodeframe2.UnpackData(bytes,length)) return false;
 if(!(pub_nodeframe2 || stamped_nodeframe2)) return true;
 nlink_parser::LinktrackNodeframe2 msg_data;
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

 if(pub_nodeframe2) pub_nodeframe2.publish(msg_data);
 if(stamped_nodeframe2) { nlink_parser::LinktrackNodeframe2Stamped stamped; stamped.context=context; stamped.frame=msg_data; stamped_nodeframe2.publish(stamped); }
 return true; }
case 5: {
 if(!g_nlt_nodeframe3.UnpackData(bytes,length)) return false;
 if(!(pub_nodeframe3 || stamped_nodeframe3)) return true;
 nlink_parser::LinktrackNodeframe3 msg_data;
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

 if(pub_nodeframe3) pub_nodeframe3.publish(msg_data);
 if(stamped_nodeframe3) { nlink_parser::LinktrackNodeframe3Stamped stamped; stamped.context=context; stamped.frame=msg_data; stamped_nodeframe3.publish(stamped); }
 return true; }
case 6: {
 if(!g_nlt_nodeframe4.UnpackData(bytes,length)) return false;
 if(!(pub_nodeframe4)) return true;
 nlink_parser::LinktrackNodeframe4 msg_data;
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

 if(pub_nodeframe4) pub_nodeframe4.publish(msg_data);
 return true; }
case 8: {
 if(!g_nlt_nodeframe5.UnpackData(bytes,length)) return false;
 if(!(pub_nodeframe5 || stamped_nodeframe5)) return true;
 nlink_parser::LinktrackNodeframe5 msg_data;
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

 if(pub_nodeframe5) pub_nodeframe5.publish(msg_data);
 if(stamped_nodeframe5) { nlink_parser::LinktrackNodeframe5Stamped stamped; stamped.context=context; stamped.frame=msg_data; stamped_nodeframe5.publish(stamped); }
 return true; }
case 9: {
 if(!g_nlt_nodeframe6.UnpackData(bytes,length)) return false;
 if(!(pub_nodeframe6 || stamped_nodeframe6)) return true;
 nlink_parser::LinktrackNodeframe6 msg_data;
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

 if(pub_nodeframe6) pub_nodeframe6.publish(msg_data);
 if(stamped_nodeframe6) { nlink_parser::LinktrackNodeframe6Stamped stamped; stamped.context=context; stamped.frame=msg_data; stamped_nodeframe6.publish(stamped); }
 return true; }
default:return false; } }
}
