# Writing an action server and client



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
- 
##
##

Try running our action server now:

```
python3 fibonacci_action_server.py
```

In another terminal, we can use the command line interface to send a goal:

```
ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```

In the terminal that is running the action server, you should see a logged message “Executing goal…” followed by a warning that the goal state was not set. We can use the method succeed() on the goal handle to indicate that the goal was successful:

```
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        **goal_handle.succeed()**

        result = Fibonacci.Result()
        return result
```

Now if you restart the action server and send another goal, you should see the goal finished with the status SUCCEEDED.

##
##

Now let’s make our goal execution actually compute and return the requested Fibonacci sequence:

```
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')


        sequence = [0, 1]



        for i in range(1, goal_handle.request.order):

            sequence.append(sequence[i] + sequence[i-1])


        goal_handle.succeed()

        result = Fibonacci.Result()

        result.sequence = sequence

        return result
```

Restart the action server and send another goal.

##

## 1.2 Publishing feedback

Now actions is able to provide feedback to an action client during goal execution. We can make our action server publish feedback for action clients by calling the goal handle’s publish_feedback() method.

Repace Yellow lines of code:

![image](https://user-images.githubusercontent.com/90166739/195600779-bdb31269-1915-4e15-abde-cc54ea9d6afe.png)

After restarting the action server, we can confirm that feedback is now published by using the command line tool with the --feedback option:

```
ros2 action send_goal --feedback fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```


## 2 Writing an action client

Open a new file and call it fibonacci_action_client.py, and add the following code:

```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    future = action_client.send_goal(10)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
```

We create an ActionClient by passing it three arguments:

- A ROS 2 node to add the action client to: self

- The type of the action: Fibonacci

- The action name: 'fibonacci'

#

Let’s test our action client by first running the action server built earlier:

```
python3 fibonacci_action_server.py
```

#

In another terminal, run the action client:

```
python3 fibonacci_action_client.py
```

You should see messages printed by the action server as it successfully executes the goal:

```
[INFO] [fibonacci_action_server]: Executing goal...
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3])
[INFO] [fibonacci_action_server]: Feedback: array('i', [0, 1, 1, 2, 3, 5])
# etc.
```

The action client should start up, and then quickly finish. It means we have a functioning action client, but not functioning action server

## 2.1 Getting a result

So we can send a goal, but how do we know when it is completed? We can get the result information with a couple steps:
1. we need to get a goal handle for the goal we sent
2. we can use the goal handle to request the result.

Here’s the complete code for this example:

```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

With an action server running in a separate terminal try running our Fibonacci action client:

```
python3 fibonacci_action_client.py
```

You should see logged messages for the goal being accepted and the final result.


## 2.2 Getting feedback

Our action client can send goals. And now we need to get some feedback about the goals we send from the action server.

Here’s the complete code for this example:

```
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

Now if we run our action client, you should see feedback being printed to the screen.

## Summary

In this tutorial, we put together a Python action server and action client line by line, and configured them to exchange goals, feedback, and results.

