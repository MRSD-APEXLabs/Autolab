// ros2_mqtt_bridge serializes every ROS message as JSON, so a std_msgs/String arrives as {"data": "<string>"}.
export function unwrapString(payload) {
  const outer = JSON.parse(payload);
  if (!outer || typeof outer !== 'object' || typeof outer.data !== 'string') {
    throw new Error('expected a bridged std_msgs/String {"data": "..."}');
  }
  return outer.data;
}
