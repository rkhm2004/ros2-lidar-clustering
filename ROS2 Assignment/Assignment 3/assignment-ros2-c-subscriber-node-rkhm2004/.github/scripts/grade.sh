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

# 3. Start the subscriber node in background
echo "Starting subscriber node..."
ros2 run ros2_subscriber subscriber_node > output.log 2>&1 &
SUBSCRIBER_PID=$!
sleep 2

# 4. Publish a test message
echo "Publishing test message..."
ros2 topic pub /chatter std_msgs/msg/String "data: 'TestMessage123'" --once

# 5. Wait a bit for the message to be received
sleep 2

# 6. Kill the subscriber
kill $SUBSCRIBER_PID 2>/dev/null || true
wait $SUBSCRIBER_PID 2>/dev/null || true

# 7. Check if the message was received
if grep -q "I heard:" output.log && grep -q "TestMessage123" output.log; then
  echo "✅ Subscriber correctly received and logged the message"
  cat output.log
  exit 0
else
  echo "❌ Subscriber did not correctly receive or log the message"
  echo "Output was:"
  cat output.log
  exit 1
fi
