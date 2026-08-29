#pragma once

/// ROS-free route-follower state shared by the C++ follower node and its tests.
///
/// Keeping the state machine outside rclcpp makes every safety latch and
/// command update independently testable. px4_vio_bridge/path_follower.py is
/// the legacy Python counterpart and is no longer held to this file.
///
/// Two invariants are load-bearing here:
///
///  * the accepted route, the commanded displacement and the command velocity
///    are all stored in continuous VIO coordinates and rendered into whichever
///    map solution the caller supplies, so a loop-closure correction moves the
///    pose and the route together instead of looking like cross-track error;
///  * a command chord must clear the full configured hard clearance, with the
///    single exception of an escape that begins from a pose already inside it.

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "px4_vio_bridge/path_geometry.hpp"

namespace px4_vio_bridge
{

struct FollowResult
{
  std::string status;
  bool valid{};
  Point2 desired_carrot{};
  Point2 commanded_carrot{};
  Point2 commanded_displacement{};
  /// The same command in continuous VIO coordinates: the quantity the flight
  /// adapter consumes, carried here so no caller has to re-derive it.
  Point2 vio_displacement{};
  double path_progress{};
  double progress{};
  double remaining{};
  double cross_track{};
  std::int32_t generation{};
};

bool requested_goal_reached(const std::string & follower_status, bool goal_terminal);

/// Minimum continuous clearance of the chord start->end, or no value when the
/// chord leaves known, in-bounds space. segment_minimum_clearance() bound to a
/// grid is the production implementation.
using ClearanceProbe =
  std::function<std::optional<double>(const Point2 & start, const Point2 & end)>;

struct ClearanceEscapeLimits
{
  double required_clearance{};
  /// How much closer to safety a sub-clearance escape's endpoint must get
  /// before it counts as recovery rather than following the same unsafe
  /// contour sideways for ever.
  double minimum_improvement{0.01};
  /// Floating-point noise only. Never a grid cell.
  double tolerance{1.0e-6};
};

struct LookaheadSelection
{
  double lookahead{};
  /// The pose is already inside the hard clearance and this chord is an escape.
  bool escaping{};
  double start_clearance{};
  /// Endpoint clearance while escaping; the chord minimum otherwise.
  double end_clearance{};
};

/// Which end of the follower's command a chord is.
///
/// The two differ only while escaping, and only in the anti-stagnation half of
/// the rule. `minimum_improvement` asks "does this target lead anywhere better",
/// which is a property of the *selected lookahead*. The acceleration-limited
/// carrot is a partial step towards a target that has already answered yes; at
/// 0.30 m/s^2 its first step is millimetres, so demanding a centimetre of gain
/// from it would forbid every escape from ever starting. Both roles enforce the
/// safety half in full: no point of the chord may be closer to an occupied cell
/// than the pose already is.
enum class ChordRole
{
  /// A candidate lookahead target: must also improve materially.
  Target,
  /// The acceleration-limited carrot actually commanded this tick.
  IntermediateCarrot,
};

/// Whether pose->target may be commanded.
///
/// At or above `required_clearance` this is exactly
/// segment_has_clearance(pose, target, required_clearance) for either role: the
/// normal hard envelope is never relaxed.
bool command_chord_admissible(
  const ClearanceProbe & probe, const Point2 & pose, const Point2 & target,
  const ClearanceEscapeLimits & limits, double start_clearance,
  ChordRole role = ChordRole::Target,
  double * end_clearance = nullptr);

/// Farthest admissible lookahead in [min_lookahead, lookahead], stepping down
/// by `lookahead_step`. No value means hold: neither a normal chord nor an
/// escape exists.
std::optional<LookaheadSelection> select_safe_lookahead(
  const Polyline & path, const Point2 & pose, const ClearanceProbe & probe,
  const ClearanceEscapeLimits & limits,
  double lookahead, double lookahead_step, double min_lookahead);

/// Whether a newly published path must wait for a settling correction episode.
///
/// Never defer when there is no route to protect. Deferring the *first* path
/// leaves the follower with nothing to fly, and the episode that caused it can
/// then never clear: `CorrectionReplanGate::waiting()` is what settles an
/// episode, and a follower with no route reports WAITING_FOR_PATH and never
/// reaches it. Flights 20260829T102157Z and 102221Z aborted on exactly that
/// latch -- the planner published PATH_VALID for eleven seconds while the
/// follower deferred every one of those paths.
bool defer_path_for_correction(bool episode_pending, bool have_route);

/// Coalesce noisy correction samples into infrequent replan barriers.
///
/// The native correction can move by centimetres between SLAM updates, so the
/// raw signal is filtered and one *episode* is opened when a material step is
/// crossed. The episode settles only after the correction has been quiet for
/// `quiet_time` and a path planned from a map received after the last material
/// change has arrived -- a path receipt alone is not enough, because a path
/// built from the pre-correction grid can arrive at any moment. Every further
/// material step restarts the quiet timer instead of being swallowed, which an
/// unconditional multi-second cooldown could not do.
///
/// The filter decides whether to *open* an episode; it must not decide when one
/// has settled. "Still moving" is asked of the RAW correction, because a
/// filtered signal keeps moving on its own long after the vehicle's estimate
/// has stopped. Measured 20260829T103521Z: the raw correction is a staircase --
/// five loop closures in 26 s, one of them 333 mm -- but comparing the lagging
/// filtered value against a reference taken from that same lagging value turned
/// each step into roughly seven self-inflicted "material changes" spread over
/// 1.5 s. Only 2 of 19 gaps reached the 0.40 s quiet time, so episodes almost
/// never settled, path acceptance stalled, and the route aged until cross-track
/// faulted.
class CorrectionReplanGate
{
public:
  CorrectionReplanGate(
    double translation_trigger = 0.05,
    double yaw_trigger = 0.026179938779914945,
    double filter_time_constant = 0.35,
    double material_translation = 0.03,
    double material_yaw = 0.013089969389957472,
    double quiet_time = 0.40,
    double rearm_guard = 0.20);

  /// Returns true when this sample opened a new episode.
  bool observe(const Correction4 & correction, double now);
  /// One occupancy grid published by the planner.
  void map_received(std::int64_t map_generation, double now);
  /// One /planner/path message.
  void path_received(double now);
  /// The map generation the newest published path was planned from.
  void path_map_generation(std::int64_t map_generation, double now);
  bool waiting(double now);

  [[nodiscard]] bool pending() const {return pending_;}
  [[nodiscard]] std::int64_t epoch() const {return epoch_;}
  [[nodiscard]] const Point2 & last_trigger_delta() const {return last_trigger_delta_;}
  [[nodiscard]] std::optional<std::int64_t> required_map_generation() const
  {
    return required_map_generation_;
  }
  /// Why the episode has not settled yet, for the status line.
  [[nodiscard]] std::string pending_detail(double now) const;

private:
  void restart_receipts();

  double translation_trigger_;
  double yaw_trigger_;
  double filter_time_constant_;
  double material_translation_;
  double material_yaw_;
  double quiet_time_;
  double rearm_guard_;
  std::optional<Correction4> filtered_;
  std::optional<Correction4> baseline_;
  // Raw, not filtered: the reference the "still moving" test uses.
  std::optional<Correction4> material_reference_;
  // The newest raw sample, so a settling episode can hand the filter the value
  // it was converging towards instead of baselining on its lag.
  std::optional<Correction4> last_raw_;
  std::optional<double> last_observation_;
  double last_material_change_{};
  bool path_after_change_{false};
  bool path_generation_after_change_{false};
  bool generation_pairing_seen_{false};
  std::optional<std::int64_t> required_map_generation_;
  bool pending_{false};
  std::int64_t epoch_{0};
  double rearm_until_{};
  Point2 last_trigger_delta_{};
};

class PositionRouteFollower
{
public:
  using CommandValidator = std::function<bool(const Point2 &)>;

  PositionRouteFollower(
    double lookahead = 0.60,
    double max_carrot_speed = 0.10,
    double max_carrot_acceleration = 0.30,
    double max_cross_track = 0.60,
    double cross_track_resume = 0.05,
    double cross_track_recovery_time = 1.0,
    double arrival_tolerance = 0.12,
    double arrival_release_tolerance = 0.20);

  void clear_path();
  void reset_route_progress();
  void interrupt_cross_track_recovery();
  void hold_command();
  /// Latch the clearance-escape mode.
  ///
  /// The rising edge drops the stale relative command so a displacement aimed
  /// at the obstacle cannot survive into the escape -- but ONLY when
  /// `stale_command_admissible` is false. A pose sitting exactly on the
  /// clearance threshold re-crosses it every tick from measurement noise, and
  /// an unconditional edge-triggered wipe then destroys the accumulating
  /// command roughly ten times a second: flight 20260829T084036Z held
  /// progress=0.00m for six seconds that way and was landed on cross-track.
  /// The command is re-validated by command_chord_admissible() on every tick
  /// regardless, so dropping one that still passes buys no safety.
  /// Staying in, or leaving, the escape always preserves route progress.
  void set_escape(bool escaping, bool stale_command_admissible = false);

  /// `correction` is the map<-vio transform the supplied map-frame points and
  /// pose belong to. The route is stored in VIO coordinates, so re-publishing
  /// the same physical route under a new correction is not a new generation.
  bool set_path(
    const std::vector<Point2> & points, const Point2 & pose,
    const Correction4 & correction = kIdentityCorrection);
  /// Re-express the accepted route in `correction` without touching progress,
  /// the fingerprint or the generation. update() does this itself; callers that
  /// need path() in the current map solution before updating call it directly.
  void rebase(const Correction4 & correction) {render(correction);}
  FollowResult update(
    const Point2 & pose, double dt,
    std::optional<double> lookahead = std::nullopt,
    const CommandValidator & command_validator = {},
    const Correction4 & correction = kIdentityCorrection);

  /// The accepted route rendered in the most recently supplied correction.
  [[nodiscard]] const Polyline * path() const {return path_.get();}
  [[nodiscard]] const std::vector<Point2> & vio_points() const {return vio_points_;}
  [[nodiscard]] const Correction4 & correction() const {return correction_;}
  [[nodiscard]] double lookahead() const {return lookahead_;}
  [[nodiscard]] double progress() const {return progress_;}
  [[nodiscard]] double path_progress() const {return path_progress_;}
  [[nodiscard]] Point2 commanded_displacement() const;
  [[nodiscard]] Point2 command_velocity() const;
  [[nodiscard]] const Point2 & vio_displacement() const {return commanded_displacement_vio_;}
  [[nodiscard]] std::int32_t generation() const {return generation_;}
  [[nodiscard]] bool cross_track_latched() const {return cross_track_latched_;}
  [[nodiscard]] bool at_goal() const {return at_goal_;}
  [[nodiscard]] bool escaping() const {return escaping_;}

private:
  /// Re-express the stored VIO route in `correction`. A pure coordinate change
  /// never touches progress, the fingerprint or the generation.
  void render(const Correction4 & correction);

  double lookahead_;
  double max_carrot_speed_;
  double max_carrot_acceleration_;
  double max_cross_track_;
  double cross_track_resume_;
  double cross_track_recovery_time_;
  double arrival_tolerance_;
  double arrival_release_tolerance_;
  std::vector<Point2> vio_points_;
  Correction4 correction_{kIdentityCorrection};
  std::unique_ptr<Polyline> path_;
  PathFingerprint fingerprint_;
  std::int32_t generation_{};
  double progress_{};
  double path_progress_{};
  Point2 commanded_displacement_vio_{};
  Point2 command_velocity_vio_{};
  bool cross_track_latched_{false};
  std::int32_t cross_track_fault_generation_{};
  double cross_track_recovery_elapsed_{};
  bool at_goal_{false};
  bool escaping_{false};
};

}  // namespace px4_vio_bridge
