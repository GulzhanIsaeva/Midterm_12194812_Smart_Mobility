# Contents:

- Writing an action server and client
- Composing multiple nodes in a single process

# Writing an action server and client

Action clients send goal requests to action servers. Action servers send goal feedback and results to action clients.

## 1 Writing an action server

Open a new file in your home directory using the code below:
```
gedit fibonacci_action_server.py
```

And call it fibonacci_action_server.py, and add the following code:

```
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Fibonacci.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
```

An action server requires four arguments:

- A ROS 2 node to add the action client to: self.

- The type of the action: Fibonacci (imported in line 5).

- The action name: 'fibonacci'.

- A callback function for executing accepted goals: self.execute_callback. This callback must return a result message for the action type.


Try running our action server now:

```
python3 fibonacci_action_server.py
```

In another terminal, we can use the command line interface to send a goal:

```
ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```

## 1.2 Publishing feedback
## 2 Writing an action client
## 2.1 Getting a result
## 2.2 Getting feedback
## Summary


##
##



# Composing multiple nodes in a single process
