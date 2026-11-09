from types import SimpleNamespace
from unittest.mock import patch
import threading
import time

import numpy as np
import pytest

from galaxea_r1pro.galaxea_r1pro_0.utils.tf_buffer import TransformBuffer
from galaxea_r1pro.galaxea_r1pro_0.transport.process_sub import get_transformer_martix
from galaxea_r1pro.galaxea_r1pro_0.utils.errors import MessageNotFoundError
from galaxea_r1pro.galaxea_r1pro_0.utils.Connect_Ros import ConnectRos
from galaxea_r1pro.galaxea_r1pro_0.backends.real import RealBackend
from galaxea_r1pro import RobotProfile, BackendKind


def tf(parent, child, xyz, quat=(0,0,0,1), stamp=1):
    return {'transforms': [{'header': {'frame_id': parent, 'stamp': {'sec': stamp}},
        'child_frame_id': child, 'transform': {'translation': dict(zip('xyz',xyz)), 'rotation': dict(zip('xyzw',quat))}}]}


def test_accumulate_static_and_dynamic_in_both_directions():
    b = TransformBuffer()
    b.update(tf('tcp','camera',[1,0,0]), static=True)
    b.update(tf('base','tcp',[0,2,0], (0,0,np.sin(np.pi/4),np.cos(np.pi/4))))
    b.update(tf('world','base',[0,0,3]))
    forward = b.lookup('/camera','world',1)
    np.testing.assert_allclose(forward[:3,3], [0,3,3], atol=1e-12)
    np.testing.assert_allclose(b.lookup('world','camera',1) @ forward, np.eye(4), atol=1e-12)
    np.testing.assert_allclose(b.lookup('camera','camera'), np.eye(4))


def test_dynamic_expiry_static_survival_and_repeated_stamp():
    b = TransformBuffer()
    with patch('time.monotonic',return_value=10):
        b.update(tf('base','tcp',[0,0,0]))
        b.update(tf('tcp','camera',[1,0,0]),static=True)
    with patch('time.monotonic',return_value=12):
        b.update(tf('base','tcp',[0,0,0]))  # Replay of same stamp, not fresh.
        with pytest.raises(LookupError):b.lookup('camera','base',1)
        np.testing.assert_allclose(b.lookup('camera','tcp',1)[:3,3], [1,0,0])
        b.update(tf('base','tcp',[0,2,0],stamp=2))
        np.testing.assert_allclose(b.lookup('camera','base',1)[:3,3], [1,2,0])
    b.clear()
    with pytest.raises(LookupError):b.lookup('camera','base')


def test_sdk_query_waits_for_chain_and_respects_timeout():
    b = TransformBuffer()
    ros = SimpleNamespace(tf_buffer=b)
    def later():
        time.sleep(.02)
        b.update(tf('base','camera',[1,2,3]),static=True)
    worker = threading.Thread(target=later);worker.start()
    try:
        result = get_transformer_martix(ros,'camera','base',timeout=.5,max_age_s=1)
        np.testing.assert_allclose(result['position'], [1,2,3])
    finally:worker.join()
    begin=time.monotonic()
    with pytest.raises(MessageNotFoundError):
        get_transformer_martix(ros,'missing','base',timeout=.03,max_age_s=1)
    assert time.monotonic()-begin < .3


def test_registration_callbacks_accumulate_tf():
    c = object.__new__(ConnectRos)
    c.connect_flag=True;c.latest_messages={};c.message_received_at={};c.tf_buffer=TransformBuffer()
    callbacks={}
    c.create_subscriber=lambda topic, typ, callback: callbacks.update({topic:callback})
    c.register_topics({'subscribers': {'/tf':'tf2_msgs/TFMessage','/tf_static':'tf2_msgs/TFMessage'}})
    callbacks['/tf_static'](tf('tcp','camera',[1,0,0]))
    callbacks['/tf'](tf('base','tcp',[0,2,0]))
    callbacks['/tf'](tf('world','base',[0,0,3]))
    np.testing.assert_allclose(c.tf_buffer.lookup('camera','world',1)[:3,3], [1,2,3])


def test_multiple_camera_topics_register_and_capture_correct_sensor():
    from unittest.mock import Mock
    profile=RobotProfile(robot_id='test',backend=BackendKind.REAL,firmware_profile='test',
        endpoint='ws://localhost:9091',options={'camera_topics': {'wrist': {'rgb8':'/wrist/rgb', '32FC1':'/wrist/depth'}}})
    backend=RealBackend(profile)
    connection=Mock(connect_flag=True)
    connection.client.is_connected=True
    connection.latest_messages={'/wrist/rgb': {'header': {'frame_id':'wrist_optical','stamp':{'sec':4}}}}
    with patch('galaxea_r1pro.galaxea_r1pro_0.backends.real.ConnectRos',return_value=connection):
        backend.connect()
    topics=connection.register_topics.call_args.args[0]['subscribers']
    assert '/wrist/rgb' in topics and '/wrist/depth' in topics
    with patch('galaxea_r1pro.galaxea_r1pro_0.backends.real.decode_color_message',return_value=np.zeros((2,2,3),dtype=np.uint8)):
        frame=backend.capture('wrist','rgb8')
    assert frame.sensor_id=='wrist' and frame.frame_id=='wrist_optical'
    assert frame.metadata['topic']=='/wrist/rgb'
    assert frame.metadata['ros_stamp']=={'sec':4}


def test_capture_rejects_stale_image():
    from unittest.mock import Mock
    from galaxea_r1pro.galaxea_r1pro_0.backends import real
    backend=RealBackend(RobotProfile(robot_id='test',backend=BackendKind.REAL,firmware_profile='test'))
    connection=Mock(connect_flag=True)
    connection.client.is_connected=True
    topic='/camera/camera/color/image_raw/compressed'
    connection.latest_messages={topic:{'header':{'frame_id':'optical'}}}
    connection.message_received_at={topic:time.monotonic()-10}
    backend.ros_connection=connection;backend._connected=True
    with patch.object(real,'IMAGE_WAIT_TIMEOUT',.01):
        with pytest.raises(MessageNotFoundError):backend.capture('head_camera','rgb8')
