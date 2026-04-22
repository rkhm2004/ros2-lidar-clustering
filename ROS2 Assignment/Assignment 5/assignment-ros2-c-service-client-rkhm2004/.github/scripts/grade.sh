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

# 3. Create a simple service server for testing
echo "Creating test service server..."
cat > /tmp/test_server.py << 'EOF'
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class TestServer(Node):
    def __init__(self):
        super().__init__('test_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.callback)
        self.get_logger().info('Test server ready')

    def callback(self, request, response):
        response.sum = request.a + request.b
        return response

def main():
    rclpy.init()
    node = TestServer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
EOF

# 4. Start the test server in background
python3 /tmp/test_server.py &
SERVER_PID=$!
sleep 3

# 5. Run the client and capture output
echo "Running client..."
timeout 10s ros2 run ros2_service_client add_two_ints_client > output.log 2>&1 || true

# 6. Kill the server
kill $SERVER_PID 2>/dev/null || true
wait $SERVER_PID 2>/dev/null || true

# 7. Check the output
echo "Client output:"
cat output.log

if grep -q "Result:" output.log && grep -q "42" output.log; then
  echo "✅ Client correctly called service and got result 42"
  exit 0
else
  echo "❌ Client did not get correct result"
  exit 1
fi
