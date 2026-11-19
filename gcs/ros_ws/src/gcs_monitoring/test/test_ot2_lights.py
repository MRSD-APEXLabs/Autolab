from unittest.mock import Mock, call
from gcs_monitoring.ot2_lights import RailLights


def test_get_only_reads_without_changing_the_robot():
    request=Mock(return_value={'on':False}); publish=Mock()
    RailLights(publish,request).handle(b'{"action":"get"}')
    assert request.call_args_list == [call()]
    assert publish.call_args.args[0] == {'on':False,'busy':False,'error':None}


def test_set_posts_boolean_and_reads_back_actual_state():
    request=Mock(side_effect=[{'on':True},{'on':True}]);publish=Mock()
    RailLights(publish,request).handle(b'{"action":"set","on":true}')
    assert request.call_args_list == [call({'on':True}),call()]
    assert publish.call_args.args[0]['on'] is True


def test_bad_payload_does_not_send_a_robot_request():
    request=Mock();publish=Mock()
    RailLights(publish,request).handle(b'{"action":"set","on":"true"}')
    request.assert_not_called()
    assert publish.call_args.args[0]['error']


def test_connection_error_clears_busy_and_reports_failure():
    request=Mock(side_effect=OSError('Robot unreachable'));publish=Mock()
    RailLights(publish,request).handle(b'{"action":"get"}')
    assert publish.call_args.args[0]['busy'] is False
    assert publish.call_args.args[0]['error']=='Robot unreachable'
