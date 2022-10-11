# Writing a simple service and client (Python)

##  1. Create a package

Open a new terminal and source your ROS 2 installation so that ros2 commands will work.

```
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces
```

Your terminal will return a message verifying the creation of your package py_srvcli and all its necessary files and folders.

## 1.1 Update package.xml

Add the description, maintainer email and name, and license information to package.xml.

```
<description>Python client server tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

## 1.2 Update setup.py

Add the same information to the setup.py file for the maintainer, maintainer_email, description and license fields:

```
maintainer='Your Name',
maintainer_email='you@email.com',
description='Python client server tutorial',
license='Apache License 2.0',
```

##  2. Write the service node

Inside the ros2_ws/src/py_srvcli/py_srvcli directory, create a new file called service_member_function.py and paste the following code:

```
from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
Examine the code and check for accuracy

##  3. Write the client node

Inside the ros2_ws/src/py_srvcli/py_srvcli directory, create a new file called client_member_function.py and paste the following code:

```
import sys

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Examine the code and check for accuracy

## 3.1 Add an entry point

Like the service node, you also have to add an entry point to be able to run the client node.

The entry_points field of your setup.py file should look like this:

```
entry_points={
    'console_scripts': [
        'service = py_srvcli.service_member_function:main',
        'client = py_srvcli.client_member_function:main',
    ],
},
```
I
##  4 Build and run

It’s good practice to run rosdep in the root of your workspace (ros2_ws) to check for missing dependencies before building:

```
rosdep install -i --from-path src --rosdistro foxy -y
```

Navigate back to the root of your workspace, ros2_ws, and build your new package:

```
colcon build --packages-select py_srvcli
```

Open a new terminal, navigate to ros2_ws, and source the setup files:

```
. install/setup.bash
```

Now run the service node:

```
ros2 run py_srvcli service
```

The node will wait for the client’s request.

Open another terminal and source the setup files from inside ros2_ws again. Start the client node, followed by any two integers separated by a space:

```
ros2 run py_srvcli client 2 3
```

If you chose 2 and 3, for example, the client would receive a response like this:

```
[INFO] [minimal_client_async]: Result of add_two_ints: for 2 + 3 = 5
```
![client node](https://user-images.githubusercontent.com/90166739/193851331-bade2e14-2ac8-4a8f-8405-ceef25799028.PNG)


Return to the terminal where your service node is running. You will see that it published log messages when it received the request:

```
[INFO] [minimal_service]: Incoming request
a: 2 b: 3
```
![service node](https://user-images.githubusercontent.com/90166739/193851231-d72f716a-8650-4edb-af45-80586aa9fb72.PNG)


Enter Ctrl+C in the server terminal to stop the node from spinning.
