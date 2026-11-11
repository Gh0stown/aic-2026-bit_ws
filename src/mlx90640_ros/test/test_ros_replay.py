#!/usr/bin/env python3
"""Opt-in end-to-end ROS replay; uses its own master, never a real serial port.

source devel/setup.bash
MLX90640_ROS_INTEGRATION=1 python3 -m unittest discover \
    -s src/mlx90640_ros/test -p test_ros_replay.py -v
"""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time
import unittest
import xmlrpc.client

import numpy as np


@unittest.skipUnless(os.environ.get('MLX90640_ROS_INTEGRATION') == '1',
                     'opt in with MLX90640_ROS_INTEGRATION=1 (starts isolated ROS master)')
class RosReplayTest(unittest.TestCase):
    def test_replay_topics(self):
        with socket.socket() as sock:
            sock.bind(('127.0.0.1', 0))
            port = sock.getsockname()[1]
        uri = 'http://127.0.0.1:{}'.format(port)
        environment = dict(os.environ, ROS_MASTER_URI=uri, ROS_IP='127.0.0.1')
        environment.pop('ROS_HOSTNAME', None)
        old_environment = dict(os.environ)
        processes = []
        subscribers = []
        with tempfile.TemporaryDirectory(prefix='mlx90640-ros-test-') as work:
            environment.update(ROS_HOME=work, ROS_LOG_DIR=str(Path(work) / 'log'))
            os.environ.clear()
            os.environ.update(environment)
            with open(Path(work) / 'processes.log', 'w+') as logfile:
                try:
                    master = subprocess.Popen(['roscore', '-p', str(port)], env=environment,
                                              stdout=logfile, stderr=subprocess.STDOUT,
                                              start_new_session=True)
                    processes.append(master)
                    # Readiness polling is bounded and checks actual master RPC.
                    deadline = time.monotonic() + 10
                    while time.monotonic() < deadline:
                        if master.poll() is not None:
                            self.fail('isolated roscore exited early')
                        try:
                            with xmlrpc.client.ServerProxy(uri) as proxy:
                                response = proxy.getPid('/mlx_replay_test')
                            if response[0] == 1:
                                break
                        except (OSError, xmlrpc.client.Error):
                            pass
                        time.sleep(.1)
                    else:
                        self.fail('isolated ROS master did not become ready')
                    import rospy
                    from sensor_msgs.msg import Image, Temperature
                    from std_msgs.msg import String
                    rospy.init_node('mlx_replay_test', anonymous=True, disable_signals=True)
                    received = {'temperature_image': [], 'thermal_image': [],
                                'ambient_temperature': [], 'hotspot': []}
                    lock = threading.Lock()
                    done = threading.Event()

                    def callback(message, topic):
                        with lock:
                            received[topic].append(message)
                            if all(len(messages) >= 2 for messages in received.values()):
                                done.set()

                    for topic, message_type in [('temperature_image', Image),
                                                ('thermal_image', Image),
                                                ('ambient_temperature', Temperature),
                                                ('hotspot', String)]:
                        subscribers.append(rospy.Subscriber('/mlx90640/' + topic, message_type,
                                                            callback, callback_args=topic))
                    process = subprocess.Popen(['roslaunch', 'mlx90640_ros', 'replay.launch',
                                                'loop:=true'], env=environment,
                                               stdout=logfile, stderr=subprocess.STDOUT,
                                               start_new_session=True)
                    processes.append(process)
                    self.assertTrue(done.wait(12), 'did not receive two batches of all four topics')
                    with lock:
                        messages = {topic: list(values) for topic, values in received.items()}
                    by_stamp = [{message.header.stamp.to_nsec(): message for message in values}
                                for values in (messages['temperature_image'],
                                               messages['thermal_image'],
                                               messages['ambient_temperature'])]
                    shared = set(by_stamp[0]).intersection(by_stamp[1], by_stamp[2])
                    self.assertGreaterEqual(len(shared), 2)
                    for image in messages['temperature_image'][:2]:
                        self.assertEqual((image.height, image.width, image.encoding, image.step),
                                         (24, 32, '32FC1', 128))
                        self.assertEqual(len(image.data), 3072)
                        self.assertEqual(image.is_bigendian, 0)
                        self.assertEqual(image.header.frame_id, 'mlx90640_link')
                        values = np.frombuffer(image.data, dtype='<f4')
                        self.assertTrue(np.isfinite(values).all())
                        extrema = (float(values.max()), float(values.min()))
                        references = [(33.04648, 21.29190), (32.84659, 21.37926)]
                        self.assertTrue(any(np.allclose(extrema, reference, atol=.05, rtol=0)
                                            for reference in references), extrema)
                    for image in messages['thermal_image'][:2]:
                        self.assertEqual((image.height, image.width, image.encoding, image.step),
                                         (330, 384, 'bgr8', 1152))
                        self.assertEqual(len(image.data), image.step * image.height)
                    for message in messages['ambient_temperature'][:2]:
                        self.assertAlmostEqual(message.temperature, 29.56, delta=.2)
                        self.assertEqual(message.variance, 0)
                    for message in messages['hotspot'][:2]:
                        payload = json.loads(message.data)
                        r0, c0, r1, c1 = payload['box']
                        self.assertTrue(0 <= r0 <= r1 <= 23)
                        self.assertTrue(0 <= c0 <= c1 <= 31)
                        self.assertGreaterEqual(payload['peak_c'], payload['mean_c'])
                        self.assertEqual(payload['floor'], '')
                    published = dict(rospy.get_published_topics())
                    self.assertNotIn('/cmd_vel', published)
                finally:
                    for subscriber in subscribers:
                        subscriber.unregister()
                    if 'rospy' in locals():
                        rospy.signal_shutdown('isolated replay test complete')
                    # Stop roslaunch first, then its isolated master, including descendants.
                    for process in reversed(processes):
                        if process.poll() is None:
                            os.killpg(process.pid, signal.SIGINT)
                            try:
                                process.wait(timeout=8)
                            except subprocess.TimeoutExpired:
                                os.killpg(process.pid, signal.SIGKILL)
                                process.wait(timeout=3)
                    logfile.seek(0)
                    output = logfile.read()
                    if not all(len(values) >= 2 for values in locals().get('received', {}).values()) or 'received' not in locals():
                        print(output)
                    os.environ.clear()
                    os.environ.update(old_environment)


if __name__ == '__main__':
    unittest.main()
