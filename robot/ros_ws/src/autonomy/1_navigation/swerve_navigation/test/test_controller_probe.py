import importlib.util
from pathlib import Path


def test_missing_bluetooth_event_node_is_distinguished_from_keyboard(tmp_path):
    script = Path(__file__).resolve().parents[2]/'scripts/check_controller.py'
    spec = importlib.util.spec_from_file_location('controller_probe', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    sys_root, dev_root = tmp_path/'sys', tmp_path/'dev'
    dev_root.mkdir()
    for event, name in [('event1', 'Keyboard'), ('event10', '8BitDo Pro 2')]:
        path = sys_root/event/'device'
        path.mkdir(parents=True)
        (path/'name').write_text(name)
    (dev_root/'event1').touch()
    assert module.missing_gamepad_nodes(sys_root, dev_root) == [(str(dev_root/'event10'), '8BitDo Pro 2')]
    (dev_root/'event10').touch()
    assert not module.missing_gamepad_nodes(sys_root, dev_root)
