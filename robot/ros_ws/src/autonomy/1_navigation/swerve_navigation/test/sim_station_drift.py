"""Test-only one-shot physical approach disturbance; never used by production launch."""
import json
import math
import os
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import sim_world

assert os.environ.get('SWERVE_NAV_FORCE_SIM') == '1'
assert os.environ.get('ROS_DOMAIN_ID') in ('78', '79')


class DriftWorld(sim_world.SimWorld):
    def __init__(self):
        self.phase, self.target, self.injected = '', None, False
        super().__init__()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/station_docking/result', self._mission, qos)

    def _mission(self, msg):
        result = json.loads(msg.data)
        self.phase, self.target = result['state'], result.get('target')

    def _step_base(self):
        if self.phase == 'stage_adjust' and self.target and not self.injected:
            world = self._start.compose(self._base.pose)
            if math.hypot(world.x-self.target[0], world.y-self.target[1]) < .37:
                error = sim_world.wrap_angle(world.yaw-self.target[2])
                pose = self._base.pose
                shifted = sim_world.Pose2D(pose.x, pose.y, pose.yaw+math.copysign(.16, error))
                if self._odom_pose_collides(shifted):
                    raise RuntimeError('injected drift would collide')
                self._base.pose = shifted
                self.injected = True
                self.get_logger().info('INJECTED_APPROACH_DRIFT: 0.16 rad')
        super()._step_base()


sim_world.SimWorld = DriftWorld
if __name__ == '__main__':
    sim_world.main()
