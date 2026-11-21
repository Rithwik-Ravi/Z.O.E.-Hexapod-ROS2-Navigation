#!/usr/bin/env python3
"""
test_brain_e2e.py — End-to-end integration test for the Z.O.E. Autonomous Brain.

Spawns `sensor_bridge`, `tool_dispatcher`, and `nvidia_brain`, publishes a
wake-word-prefixed query on `/brain/query`, and verifies a response arrives on
`/brain/response`. Requires an active ROS 2 environment and `NVIDIA_API_KEY`.
"""

import os
import site
import subprocess
import sys
import time

workspace = os.path.dirname(os.path.abspath(__file__))
ros_distro = os.environ.get('ROS_DISTRO', 'jazzy' if os.path.isdir('/opt/ros/jazzy') else 'humble')

secrets_file = os.path.join(workspace, '.brain_secrets')
if os.path.isfile(secrets_file):
    with open(secrets_file, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if line.startswith('NVIDIA_API_KEY='):
                os.environ['NVIDIA_API_KEY'] = line.split('=', 1)[1].strip().strip('"\'')

ros_env = os.environ.copy()
user_site = site.getusersitepackages()
ros_env['PYTHONPATH'] = f"{user_site}:{ros_env.get('PYTHONPATH', '')}"

query_runner_code = """
import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

rclpy.init()
node = Node('test_query_runner')
pub = node.create_publisher(String, '/brain/query', 10)
received = []

def on_resp(msg):
    print(f'[RESPONSE RECEIVED] {msg.data}')
    received.append(msg.data)

node.create_subscription(String, '/brain/response', on_resp, 10)
print('Test runner initialized, waiting for connections...')

for _ in range(15):
    rclpy.spin_once(node, timeout_sec=0.1)

query = 'Hey Zoe, rotate 30 degrees to inspect the room'
print(f'Sending query: {query}')
pub.publish(String(data=query))

t0 = time.time()
while time.time() - t0 < 30 and not received:
    rclpy.spin_once(node, timeout_sec=0.2)

print(f'Total responses received: {len(received)}')
rclpy.shutdown()
sys.exit(0 if len(received) > 0 else 1)
"""

runner_path = '/tmp/test_query_runner.py'
with open(runner_path, 'w', encoding='utf-8') as f:
    f.write(query_runner_code)

setup_cmd = f"source /opt/ros/{ros_distro}/setup.bash && source {workspace}/install/setup.bash"
procs = []
try:
    for node_exe in ['sensor_bridge', 'tool_dispatcher', 'nvidia_brain']:
        print(f'Launching {node_exe}...')
        proc = subprocess.Popen(
            ['bash', '-c', f'{setup_cmd} && ros2 run rover {node_exe}'],
            cwd=workspace,
            env=ros_env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        procs.append(proc)

    time.sleep(5)

    print('Executing query runner...')
    res = subprocess.run(
        ['bash', '-c', f'{setup_cmd} && python3 {runner_path}'],
        cwd=workspace,
        env=ros_env,
        capture_output=True,
        text=True,
        timeout=60,
    )
    print('Runner STDOUT:\n', res.stdout)
    if res.stderr:
        print('Runner STDERR:\n', res.stderr)

    if res.returncode == 0:
        print('SUCCESS: Autonomous brain end-to-end test passed!')
    else:
        print(f'Test failed with code {res.returncode}')
    sys.exit(res.returncode)

finally:
    for proc in procs:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()
