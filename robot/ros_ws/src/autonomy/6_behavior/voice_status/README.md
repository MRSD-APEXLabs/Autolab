# voice_status

Says the robot's behavior tree (BT) steps on the speaker, in a female voice (Piper
`en_US-ljspeech-high`, slightly slowed): "Moving to the
Opentrons." when an action starts, and "Arrived at the Opentrons." (or "Could not reach ...")
when it ends. It follows the same BT action topics, with the same rules, as `led_status`.

## Phrases

| BT action | Start | Success | Failure |
|---|---|---|---|
| Go To Location | Moving to {place}. | Arrived at {place}. | Could not reach {place}. |
| Navigate To Pose | Moving to the goal pose. | Arrived at the goal pose. | Could not reach the goal pose. |
| Pick Object | Picking up the well plate from {machine}. | Picked up the well plate. | Could not pick up the well plate. |
| Place Object | Placing the well plate on {machine}. | Placed the well plate on {machine}. | Could not place the well plate on {machine}. |
| Pick Base Object | Picking up the well plate from the robot. | Picked up the well plate from the robot. | Could not pick up ... |
| Place Base Object | Placing the well plate on the robot. | Placed the well plate on the robot. | Could not place ... |
| Execute OT2 Protocol | Starting the Opentrons protocol. | The Opentrons protocol is done. | The Opentrons protocol failed. |
| Execute Shaker Protocol | Starting the shaker. | The shaker is done. | The shaker failed. |

`{place}` comes from the last `go_to_location_command`. `{machine}` and the object come from the
last `manipulation_command`. Both are read when the action starts. `OT-2`/`ot2` is said as "the
Opentrons", and `home` as "Moving home." / "Arrived home.". To change a phrase, edit `phrases()`
in `voice_status/logic.py`.

## How it works

- `voice_status/logic.py`: pure phrase selection, the same activation, stale and grace rules
  as `led_status/logic.py` (see that README). Unit tested.
- `voice_status/speech.py`: [Piper](https://github.com/rhasspy/piper) (offline neural TTS,
  voice `en_US-ljspeech-high`) renders each phrase once and caches it. It is played through
  ALSA with libasound via ctypes, so the image needs no extra packages. Phrases play in order
  on a worker thread, so the BT is never blocked.
- The speaker is HDMI 0 on the Jetson: ALSA `plughw:CARD=HDA,DEV=3` (`device` launch arg). The
  HDMI sink mutes until it locks onto an audio stream, which takes most of a second. Opening
  the device per phrase therefore clipped the first word, so the node keeps the device open
  between phrases. While idle it writes noise of ±2 steps (about −84 dBFS, inaudible) rather than
  exact zeros, because the sink also mutes on digital silence and fades back in, which still
  swallowed the first sound. Nothing else can play on HDMI while the node runs, including
  `audition`: stop `robot_up` first, or `audition` reports the device as busy.

## Voice and speed

Launch args (`launch/voice_status.launch.xml`):
- `voice`: a model name in `voice/` (default `en_US-ljspeech-high`).
- `length_scale`: the speaking pace. 1.0 is the voice's normal speed, and larger values are
  slower. The default is 1.3.

To compare voices, play each downloaded voice on the speaker, one after another:

```bash
ros2 run voice_status audition                      # every voice in voice/
ros2 run voice_status audition --length-scale 1.3 en_US-amy-medium en_GB-cori-high
```

Female voices downloaded so far: `en_US-lessac-medium`, `en_US-lessac-high`, `en_US-amy-medium`,
`en_US-kristin-medium`, `en_US-hfc_female-medium`, `en_US-ljspeech-high`,
`en_GB-jenny_dioco-medium`, `en_GB-cori-high` and `en_GB-alba-medium`. More voices are listed at
https://huggingface.co/rhasspy/piper-voices. Put the `.onnx` file and its `.onnx.json` in `voice/`.
Take both files from the `v1.0.0` tag (`.../resolve/v1.0.0/...`). Several voices on `main` were
re-exported for a newer Piper, and this Piper (2023.11.14-2) rejects them with
`Phonemes must be one codepoint`.

## Setup

1. Download Piper and the voice (about 160 MB, git-ignored): `./fetch_voice.sh`.
2. The container needs the host `audio` group (gid 29, in `robot/docker/docker-compose.yaml`).
   Recreate the container once to apply it.
3. `colcon build --packages-select voice_status`. The node starts from
   `behavior_bringup/launch/behavior.launch.xml`.

## Testing

```bash
python3 -m pytest -q test/            # no ROS, no audio
# voice + speaker check, inside the container:
echo "Moving to the Opentrons." | voice/piper/piper --model voice/en_US-ljspeech-high.onnx --output_file /tmp/v.wav
# node alone, root namespace (the BT republishes _active every tick, so keep -r):
ros2 run voice_status voice_status --ros-args -p voice_dir:=$PWD/voice
ros2 topic pub -1 /go_to_location_command std_msgs/msg/String "{data: OT-2}"
ros2 topic pub -r 10 /go_to_location_active behavior_tree_msgs/msg/Active "{active: true, id: 1}" &
ros2 topic pub -r 20 /go_to_location_status behavior_tree_msgs/msg/Status "{status: 1, id: 1}"   # then status: 2
```

Every phrase is logged as `saying: "..."`. If playback fails (no voice, device busy or
missing), it is logged as `could not say "...": <reason>`.
