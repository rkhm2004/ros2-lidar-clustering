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

# 3. Check if the message type exists
echo "Checking message type..."
if ros2 interface show ros2_custom_msgs/msg/RobotStatus > /dev/null 2>&1; then
  echo "✅ Custom message type exists"
  ros2 interface show ros2_custom_msgs/msg/RobotStatus
else
  echo "❌ Custom message type not found"
  exit 1
fi

# 4. Verify message fields
MSG_DEF=$(ros2 interface show ros2_custom_msgs/msg/RobotStatus)
if echo "$MSG_DEF" | grep -q "robot_name" && \
   echo "$MSG_DEF" | grep -q "battery_level" && \
   echo "$MSG_DEF" | grep -q "is_active" && \
   echo "$MSG_DEF" | grep -q "mission_count"; then
  echo "✅ All required fields present"
else
  echo "❌ Missing required fields in message definition"
  exit 1
fi

# 5. Run the node and capture output
echo "Running status publisher..."
timeout 5s ros2 run ros2_custom_msgs status_publisher > output.log 2>&1 &
NODE_PID=$!
sleep 3

# 6. Echo the topic
ros2 topic echo /robot_status --once > topic_output.log 2>&1 || true

kill $NODE_PID 2>/dev/null || true
wait $NODE_PID 2>/dev/null || true

# 7. Check output
echo "Node output:"
cat output.log

echo "Topic output:"
cat topic_output.log

if grep -q "Explorer1" output.log && grep -q "battery" output.log; then
  echo "✅ Node publishing correct data"
  exit 0
else
  echo "❌ Node not publishing expected data"
  exit 1
fi
