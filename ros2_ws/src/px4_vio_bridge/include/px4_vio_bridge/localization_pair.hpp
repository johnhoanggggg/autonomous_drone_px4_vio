#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <optional>

namespace px4_vio_bridge
{
/// Match map body pose and map<-VIO correction by their shared source stamp.
/// Receipt order is deliberately irrelevant. Only complete, newer pairs can
/// replace a consumer's active localization; individual receipts never refresh
/// that localization's freshness. Bounded queues tolerate interleaved samples.
class LocalizationPairBuffer
{
public:
  using Pose = std::array<double, 3>;
  using Correction = std::array<double, 4>;
  struct Pair
  {
    std::int64_t stamp;
    Pose pose;
    Correction correction;
    double pose_received;
    double correction_received;
  };

  std::optional<Pair> pose(std::int64_t stamp, const Pose & value, double received)
  {
    if (stamp <= floor_) {return std::nullopt;}
    // Do not refresh age on duplicate delivery of the same source sample.
    poses_.try_emplace(stamp, Entry<Pose>{value, received});
    trim(poses_);
    return match(stamp);
  }

  std::optional<Pair> correction(
    std::int64_t stamp, const Correction & value, double received)
  {
    if (stamp <= floor_) {return std::nullopt;}
    corrections_.try_emplace(stamp, Entry<Correction>{value, received});
    trim(corrections_);
    return match(stamp);
  }

  /// Invalid input must not subsequently be bypassed by an older pending pair.
  void discard_through(std::int64_t stamp)
  {
    floor_ = std::max(floor_, stamp);
    poses_.erase(poses_.begin(), poses_.upper_bound(floor_));
    corrections_.erase(corrections_.begin(), corrections_.upper_bound(floor_));
  }

  void reject(std::int64_t stamp)
  {
    history_.erase(stamp);
    discard_through(stamp);
  }

  bool pending() const {return !poses_.empty() || !corrections_.empty();}

  /// Paths name the exact localization sample used to produce their geometry.
  /// Keep a short history so cross-topic delivery order cannot change its frame.
  std::optional<Pair> at(std::int64_t stamp) const
  {
    const auto found = history_.find(stamp);
    return found == history_.end() ? std::nullopt : std::optional<Pair>{found->second};
  }

private:
  template<typename T> struct Entry {T value; double received;};
  template<typename T> static void trim(std::map<std::int64_t, Entry<T>> & entries)
  {
    while (entries.size() > 16) {entries.erase(entries.begin());}
  }
  std::optional<Pair> match(std::int64_t stamp)
  {
    const auto p = poses_.find(stamp);
    const auto c = corrections_.find(stamp);
    if (p == poses_.end() || c == corrections_.end()) {return std::nullopt;}
    const Pair result{stamp, p->second.value, c->second.value,
      p->second.received, c->second.received};
    history_.emplace(stamp, result);
    while (history_.size() > 64) {history_.erase(history_.begin());}
    discard_through(stamp);
    return result;
  }
  std::int64_t floor_{0};
  std::map<std::int64_t, Entry<Pose>> poses_;
  std::map<std::int64_t, Entry<Correction>> corrections_;
  std::map<std::int64_t, Pair> history_;
};
}  // namespace px4_vio_bridge
