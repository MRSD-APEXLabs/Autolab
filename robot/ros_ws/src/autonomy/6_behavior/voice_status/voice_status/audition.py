"""Plays each downloaded voice on the speaker, one after another, so you can pick one.

    ros2 run voice_status audition                        # every voice in voice/, at length_scale 1.3
    ros2 run voice_status audition --length-scale 1.3 en_US-amy-medium en_GB-cori-high
"""
import argparse
import glob
import os

from voice_status.speech import AlsaStream, PiperVoice

VOICE_DIR = '/home/robot/AutoLab/robot/ros_ws/src/autonomy/6_behavior/voice_status/voice'


def main():
    parser = argparse.ArgumentParser(description='Play each Piper voice so you can choose one.')
    parser.add_argument('voices', nargs='*', help='voice names (default: every .onnx in --voice-dir)')
    parser.add_argument('--voice-dir', default=VOICE_DIR)
    parser.add_argument('--length-scale', type=float, default=1.3, help='1.0 = normal pace, larger = slower')
    parser.add_argument('--text', default='Moving to the Opentrons. Arrived at the Opentrons.')
    parser.add_argument('--device', default='plughw:CARD=HDA,DEV=3')
    args = parser.parse_args()

    voices = args.voices or sorted(os.path.basename(path)[:-len('.onnx')]
                                   for path in glob.glob(os.path.join(args.voice_dir, '*.onnx')))
    piper = os.path.join(args.voice_dir, 'piper', 'piper')
    player = AlsaStream(args.device, print)
    for number, name in enumerate(voices, 1):
        print(f'{number}. {name}', flush=True)
        voice = PiperVoice(piper, os.path.join(args.voice_dir, name + '.onnx'), args.length_scale)
        try:
            player.play(*voice.synthesize(f'Voice {number}. {args.text}'))
        except (OSError, RuntimeError) as exc:
            print(f'   skipped: {exc}', flush=True)
    player.close()


if __name__ == '__main__':
    main()
