agma once
#include "msg/rs_msg/lidar_point_cloud_msg.hpp"

#include "utility/yaml_reader.hpp"

#include <rs_driver/msg/packet.hpp>

namespace robosense
{
namespace lidar
{

class DestinationPointCloud
{
public:
  typedef std::shared_ptr<DestinationPointCloud> Ptr;

  virtual void init(const YAML::Node& config){}
  virtual void start() {}
  virtual void stop() {}
  virtual void sendPointCloud(const LidarPointCloudMsg& msg) = 0;
  virtual ~DestinationPointCloud() = default;
};

class DestinationPacket
{
public:
  typedef std::shared_ptr<DestinationPacket> Ptr;

  virtual void init(const YAML::Node& config){}
  virtual void start() {}
  virtual void stop() {}
  virtual void sendPacket(const Packet& msg) = 0;
  virtual ~DestinationPacket() = default;
};

enum SourceType
{
  MSG_FROM_LIDAR = 1,
  MSG_FROM_ROS_PACKET = 2,
  MSG_FROM_PCAP = 3,
  MSG_FROM_PROTO_PACKET = 4,
  MSG_FROM_PROTO_POINTCLOUD = 5
};

class Source
{
public:
  typedef std::shared_ptr<Source> Ptr;

  virtual void init(const YAML::Node& config) {}
  virtual void start() {}
  virtual void stop() {}
  virtual void regPointCloudCallback(DestinationPointCloud::Ptr dst);
  virtual void regPacketCallback(DestinationPacket::Ptr dst);
  virtual ~Source() = default;
  Source(SourceType src_type);

protected:

  void sendPacket(const Packet& msg);
  void sendPointCloud(std::shared_ptr<LidarPointCloudMsg> msg);

  SourceType src_type_;
  std::vector<DestinationPointCloud::Ptr> pc_cb_vec_;
  std::vector<DestinationPacket::Ptr> pkt_cb_vec_;
};

inline Source::Source(SourceType src_type)
  : src_type_(src_type)
{
}

inline void Source::regPacketCallback(DestinationPacket::Ptr dst)
{
  pkt_cb_vec_.emplace_back(dst);
}

inline void Source::regPointCloudCallback(DestinationPointCloud::Ptr dst)
{
  pc_cb_vec_.emplace_back(dst);
}

inline void Source::sendPacket(const Packet& msg)
{
  for (auto iter : pkt_cb_vec_)
  {
    iter->sendPacket(msg);
  }
}

inline void Source::sendPointCloud(std::shared_ptr<LidarPointCloudMsg> msg)
{
  for (auto iter : pc_cb_vec_)
  {
    iter->sendPointCloud(*msg);
  }
}


}  // namespace lidar
}  // namespace robosense
