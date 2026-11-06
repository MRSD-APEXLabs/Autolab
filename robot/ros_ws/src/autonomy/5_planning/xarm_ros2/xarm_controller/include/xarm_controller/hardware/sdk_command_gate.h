#ifndef XARM_CONTROLLER_SDK_COMMAND_GATE_H_
#define XARM_CONTROLLER_SDK_COMMAND_GATE_H_

#include <atomic>
#include <cstdint>

namespace uf_robot_hardware
{
// One atomic state makes acquiring SDK ownership and claiming ROS interfaces mutually
// exclusive. The remaining ROS interfaces are still readable during SDK motion.
class SdkCommandGate
{
public:
  bool switch_claims(uint32_t stopped, uint32_t started)
  {
    auto old = state_.load();
    do {
      if ((old & SDK) && started) return false;
    } while (!state_.compare_exchange_weak(old, (old & ~stopped) | started));
    return true;
  }

  bool acquire()
  {
    uint32_t expected = 0;
    return state_.compare_exchange_strong(expected, SDK);
  }

  bool release()
  {
    uint32_t expected = SDK;
    return state_.compare_exchange_strong(expected, RESYNC);
  }

  void synchronized() { state_.fetch_and(~RESYNC); }
  bool sdk_owned() const { return (state_.load() & SDK) != 0; }
  bool resync_pending() const { return (state_.load() & RESYNC) != 0; }
  uint32_t claimed() const { return state_.load() & CLAIMS; }
  bool may_write() const
  {
    const auto state = state_.load();
    return (state & CLAIMS) && !(state & (SDK | RESYNC));
  }

private:
  static constexpr uint32_t CLAIMS = (1u << 14) - 1;
  static constexpr uint32_t SDK = 1u << 24;
  static constexpr uint32_t RESYNC = 1u << 25;
  std::atomic<uint32_t> state_{0};
};
}  // namespace uf_robot_hardware
#endif
