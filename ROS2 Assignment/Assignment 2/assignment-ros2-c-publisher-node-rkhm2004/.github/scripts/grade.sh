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

# 3. Run the node with a timeout and capture output
echo "Running publisher node..."
timeout 5s ros2 run ros2_publisher publisher_node > output.log 2>&1 || true

# 4. Check if count messages are being published
if grep -q "Count:" output.log; then
  echo "✅ Found counter messages in output"
else
  echo "❌ Could not find counter messages in output:"
  cat output.log
  exit 1
fi

# 5. Check if publishing log is present
if grep -q "Publishing:" output.log; then
  echo "✅ Found 'Publishing:' log messages"
  exit 0
else
  echo "❌ Could not find 'Publishing:' log messages"
  cat output.log
  exit 1
fi
