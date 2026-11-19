"""Independent rail-light control; does not connect, home, or move the OT-2."""
import json
import os
import time
import urllib.request

import paho.mqtt.client as mqtt

COMMAND_TOPIC = 'cmd/lab_machine/ot2_lights'
STATUS_TOPIC = 'ot2/lights/status'


def light_request(body=None):
    base = os.environ.get('OT2_HTTP_URL', 'http://192.168.10.17:31950').rstrip('/')
    request = urllib.request.Request(
        base + '/robot/lights',
        data=json.dumps(body).encode() if body is not None else None,
        headers={'Opentrons-Version': '3', 'Content-Type': 'application/json'})
    with urllib.request.urlopen(request, timeout=5) as response:
        return json.load(response)


class RailLights:
    def __init__(self, publish, request=light_request):
        self.publish, self.request = publish, request
        self.on = None

    def handle(self, payload):
        try:
            body = json.loads(payload)
            action = body.get('action')
            if action not in ('get', 'set'):
                raise ValueError('Light action must be get or set')
            if action == 'set' and type(body.get('on')) is not bool:
                raise ValueError('Light state must be true or false')
            self.publish({'on': self.on, 'busy': True, 'error': None})
            if action == 'set':
                self.request({'on': body['on']})
            state = self.request()
            if type(state.get('on')) is not bool:
                raise ValueError('Robot returned an invalid light state')
            self.on = state['on']
            self.publish({'on': self.on, 'busy': False, 'error': None})
        except Exception as exc:
            self.publish({'on': self.on, 'busy': False, 'error': str(exc)})


def main():
    client = mqtt.Client(client_id='ot2-rail-lights')
    control = RailLights(lambda state: client.publish(STATUS_TOPIC, json.dumps(state), retain=True))

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(COMMAND_TOPIC)
            control.handle(b'{"action":"get"}')

    def on_message(client, userdata, message):
        # Do not reapply an old retained setting after a reconnect.
        if not message.retain:
            control.handle(message.payload)

    client.on_connect = on_connect
    client.on_message = on_message
    host = os.environ.get('MQTT_HOST', 'localhost')
    port = int(os.environ.get('MQTT_PORT', '1883'))
    while True:
        try:
            client.connect(host, port, 60)
            break
        except OSError:
            time.sleep(2)
    try:
        client.loop_forever()
    finally:
        client.disconnect()


if __name__ == '__main__':
    main()
