/*
 * trajectory_player.cc
 *
 * Minimal Gazebo Harmonic system plugin used by the TIO-Former simulation
 * pipeline.  It replays a pre-generated CSV trajectory on a model and
 * publishes the exact model pose as ground truth.
 *
 * The simulator is used only as a ray-caster; there is no dynamics or
 * controller in the loop.  Keeping this plugin independent of Betaflight/ROS
 * makes the dataset generation reproducible and cheap.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/msgs/pose_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace tio
{
namespace sim
{

class TrajectoryPlayer
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate
{
  struct Sample
  {
    double t{0.0};
    gz::math::Pose3d pose;
  };

 public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager & /*_eventMgr*/) override
  {
    this->model = gz::sim::Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      std::cerr << "[trajectory_player] plugin must be attached to a model\n";
      return;
    }
    this->modelName = this->model.Name(_ecm);

    const std::string topic =
        _sdf->Get<std::string>("gt_topic", "/tio/gt").first;
    this->publishRate = _sdf->Get<double>("publish_rate", 15.0).first;
    std::string path = _sdf->Get<std::string>("trajectory_file", "").first;
    if (path.empty())
      path = std::getenv("TIO_TRAJECTORY_FILE") == nullptr
                 ? ""
                 : std::getenv("TIO_TRAJECTORY_FILE");

    if (path.empty() || !this->LoadCsv(path))
    {
      std::cerr << "[trajectory_player] no usable trajectory; set "
                   "TIO_TRAJECTORY_FILE or <trajectory_file>\n";
      return;
    }

    this->publisher = this->node.Advertise<gz::msgs::Pose_V>(topic);
    if (!this->publisher)
    {
      std::cerr << "[trajectory_player] failed to advertise " << topic << "\n";
      return;
    }
    this->ready = true;
    std::cerr << "[trajectory_player] loaded " << this->samples.size()
              << " samples from " << path << ", publishing on " << topic
              << "\n";
  }

 public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->ready)
      return;

    const double t = std::chrono::duration<double>(_info.simTime).count();
    const gz::math::Pose3d pose = this->SampleAt(t);
    this->model.SetWorldPoseCmd(_ecm, pose);

    if (t + 1e-9 < this->nextPublish)
      return;
    this->nextPublish += 1.0 / std::max(this->publishRate, 1e-6);
    this->Publish(t, pose);
  }

 private: gz::math::Pose3d SampleAt(double _t) const
  {
    if (_t <= this->samples.front().t)
      return this->samples.front().pose;
    if (_t >= this->samples.back().t)
      return this->samples.back().pose;

    // Samples are sorted by time; locate the bracketing pair.
    const auto upper = std::upper_bound(
        this->samples.begin(), this->samples.end(), _t,
        [](double value, const Sample &sample) { return value < sample.t; });
    const std::size_t index = static_cast<std::size_t>(
        std::max<std::ptrdiff_t>(std::distance(this->samples.begin(), upper) - 1, 0));
    const auto &a = this->samples[index];
    const auto &b = this->samples[index + 1];
    const double dt = b.t - a.t;
    const double alpha = dt > 1e-12 ? std::clamp((_t - a.t) / dt, 0.0, 1.0) : 0.0;
    const auto q = gz::math::Quaterniond::Slerp(alpha, a.pose.Rot(), b.pose.Rot(), true);
    const auto p = a.pose.Pos() + alpha * (b.pose.Pos() - a.pose.Pos());
    return gz::math::Pose3d(p, q);
  }

 private: void Publish(double _t, const gz::math::Pose3d &_pose)
  {
    gz::msgs::Pose_V msg;
    const auto ns = static_cast<int64_t>(std::llround(_t * 1e9));
    msg.mutable_header()->mutable_stamp()->set_sec(ns / 1000000000LL);
    msg.mutable_header()->mutable_stamp()->set_nsec(ns % 1000000000LL);
    auto *pose = msg.add_pose();
    pose->set_name(this->modelName);
    pose->set_id(static_cast<uint32_t>(this->model.Entity()));
    pose->mutable_position()->set_x(_pose.Pos().X());
    pose->mutable_position()->set_y(_pose.Pos().Y());
    pose->mutable_position()->set_z(_pose.Pos().Z());
    pose->mutable_orientation()->set_x(_pose.Rot().X());
    pose->mutable_orientation()->set_y(_pose.Rot().Y());
    pose->mutable_orientation()->set_z(_pose.Rot().Z());
    pose->mutable_orientation()->set_w(_pose.Rot().W());
    this->publisher.Publish(msg);
  }

 private: bool LoadCsv(const std::string &_path)
  {
    std::ifstream stream(_path);
    if (!stream)
    {
      std::cerr << "[trajectory_player] cannot open " << _path << "\n";
      return false;
    }
    std::string line;
    while (std::getline(stream, line))
    {
      if (line.empty() || line[0] == '#')
        continue;
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream row(line);
      Sample sample;
      double qw, qx, qy, qz;
      if (!(row >> sample.t >> sample.pose.Pos().X() >>
            sample.pose.Pos().Y() >> sample.pose.Pos().Z() >> qw >> qx >>
            qy >> qz))
        continue;
      sample.pose.Rot() = gz::math::Quaterniond(qw, qx, qy, qz).Normalized();
      this->samples.push_back(sample);
    }
    if (this->samples.size() < 2)
      return false;
    std::sort(this->samples.begin(), this->samples.end(),
              [](const Sample &a, const Sample &b) { return a.t < b.t; });
    return true;
  }

 private: gz::sim::Model model{gz::sim::kNullEntity};
 private: std::string modelName;
 private: std::vector<Sample> samples;
 private: bool ready{false};
 private: double publishRate{15.0};
 private: double nextPublish{0.0};
 private: gz::transport::Node node;
 private: gz::transport::Node::Publisher publisher;
};

}  // namespace sim
}  // namespace tio

GZ_ADD_PLUGIN(
    tio::sim::TrajectoryPlayer,
    gz::sim::System,
    tio::sim::TrajectoryPlayer::ISystemConfigure,
    tio::sim::TrajectoryPlayer::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(tio::sim::TrajectoryPlayer, "tio::TrajectoryPlayer")
