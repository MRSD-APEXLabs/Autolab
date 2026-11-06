#include "xarm_controller/hardware/sdk_command_gate.h"

#include <atomic>
#include <cassert>
#include <thread>

using uf_robot_hardware::SdkCommandGate;

int main()
{
  SdkCommandGate gate;
  assert(!gate.may_write());  // No controller: no periodic joint hold.
  assert(gate.switch_claims(0, 1));
  assert(gate.may_write());
  assert(!gate.acquire());
  assert(gate.switch_claims(1, 0));
  assert(!gate.may_write());
  assert(gate.acquire());
  assert(!gate.acquire());  // A second SDK client cannot acquire.
  assert(!gate.switch_claims(0, 1));  // No ROS reactivation during SDK motion.
  assert(!gate.may_write());
  assert(gate.release());
  assert(gate.resync_pending());
  assert(!gate.acquire());
  assert(gate.switch_claims(0, 1));
  assert(!gate.may_write());  // Even a claim cannot bypass measured-joint resync.
  gate.synchronized();
  assert(gate.may_write());

  for (int i = 0; i < 200; ++i) {
    SdkCommandGate concurrent;
    std::atomic<bool> go{false};
    bool sdk = false, ros = false;
    std::thread sdk_thread([&] { while (!go.load()) {} sdk = concurrent.acquire(); });
    std::thread ros_thread([&] { while (!go.load()) {} ros = concurrent.switch_claims(0, 1); });
    go.store(true);
    sdk_thread.join();
    ros_thread.join();
    assert(sdk != ros);  // Exactly one command source wins the atomic handoff.
    assert(!(concurrent.sdk_owned() && concurrent.may_write()));
  }
}
