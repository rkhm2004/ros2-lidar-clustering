#!/bin/bash

# 1. Source the ROS 2 environment
source /opt/ros/jazzy/setup.bash

# 2. Source the local workspace (if it was built)
if [ -f install/setup.bash ]; then
  source install/setup.bash
else
  echo "❌ Error: install/setup.bash not found. Did the build fail?"
  exit 1
fi

# 3. Check if launch file exists
if [ ! -f install/ros2_launch_demo/share/ros2_launch_demo/launch/pubsub.launch.py ]; then
  echo "❌ Launch file not found. Did you install the launch directory?"
  exit 1
fi
echo "✅ Launch file found"

# 4. Run the launch file and capture output
echo "Running launch file..."
timeout 8s ros2 launch ros2_launch_demo pubsub.launch.py > output.log 2>&1 || true

# 5. Check output
echo "Launch output:"
cat output.log

# Check for talker publishing with ROS2 prefix
if grep -q "ROS2:" output.log; then
  echo "✅ Talker publishing with 'ROS2' prefix (parameter set correctly)"
else
  echo "❌ Talker not publishing with 'ROS2' prefix"
  echo "Make sure you set the message_prefix parameter to 'ROS2'"
  exit 1
fi

# Check for listener receiving messages
if grep -q "I heard:" output.log; then
  echo "✅ Listener receiving messages"
  exit 0
else
  echo "❌ Listener not receiving messages"
  exit 1
fi
