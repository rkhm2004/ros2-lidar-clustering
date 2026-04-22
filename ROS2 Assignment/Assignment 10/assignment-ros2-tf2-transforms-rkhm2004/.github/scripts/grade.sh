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

# 3. Start the TF broadcaster in background
echo "Starting TF broadcaster..."
ros2 run ros2_tf2 tf_broadcaster > output.log 2>&1 &
NODE_PID=$!
sleep 2

# 4. Check if transform is being published
echo "Checking transforms..."
timeout 5s ros2 run tf2_ros tf2_echo world robot > tf_output.log 2>&1 &
TF_PID=$!
sleep 3
kill $TF_PID 2>/dev/null || true

echo "TF output:"
cat tf_output.log

# 5. Check for correct frame names
if grep -q "world" tf_output.log && grep -q "robot" tf_output.log; then
  echo "✅ Correct frame names (world -> robot)"
else
  echo "❌ Frame names not correct. Expected 'world' and 'robot'"
  kill $NODE_PID 2>/dev/null || true
  exit 1
fi

# 6. Check for translation values (should see numbers around 2.0 or less for x,y)
if grep -q "Translation:" tf_output.log; then
  echo "✅ Transform being broadcast with translation"
else
  echo "❌ No translation found in transform"
  kill $NODE_PID 2>/dev/null || true
  exit 1
fi

# 7. Cleanup
kill $NODE_PID 2>/dev/null || true
wait $NODE_PID 2>/dev/null || true

echo "✅ TF2 broadcaster working correctly"
exit 0
