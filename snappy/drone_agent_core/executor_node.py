import threading
import time
import rclpy

from geometry_msgs.msg import Twist
from px4_agent_interfaces.msg import ControlState
from px4_agent_interfaces.action import ExecutePlan
from px4_msgs.msg import VehicleLocalPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor

from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data)
from std_srvs.srv import Trigger

from drone_agent_core.executor_core import Control, ExecutorCore
from drone_agent_core.lastknown import LastKnown
from drone_agent_core.px4_mapping import to_pose
from drone_agent_core.ros_robot import POSITION_TOPIC, PX4_QOS
from drone_agent_core.limits import HOLD

AGENT_TOPIC = "/agent_velocity_cmd"
CONTROL_TOPIC = "/control_state"
POSE_TOLERANCE_S = 1.0

TICK_PERIOD_S = 0.1             # 10 Hz; the mux drops the agent after 0.5 s of silence

# The mux publishes ControlState reliable + latched: the subscriber must watch
CONTROL_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)


class ExecutorNode(Node):
    def __init__(self):
        super().__init__("snappy_executor")
        self.core = ExecutorCore()
        self.lock = threading.Lock()             # guards core, _pose and _control
        self._pose = LastKnown(POSE_TOLERANCE_S)
        self._control = Control("IDLE", False)   # not armed until the mux says so
        group = ReentrantCallbackGroup()
        self.create_subscription(VehicleLocalPosition, POSITION_TOPIC, self._on_position, PX4_QOS, callback_group=group)
        self.create_subscription(ControlState, CONTROL_TOPIC, self._on_control, CONTROL_QOS, callback_group=group)
        self._velocity_pub = self.create_publisher(Twist, AGENT_TOPIC, qos_profile_sensor_data)
        self.create_service(Trigger, "/executor/pause", self._on_pause, callback_group=group)
        self.create_service(Trigger, "/executor/resume", self._on_resume, callback_group=group)
        
        self._action = ActionServer(
            self, ExecutePlan, "execute_plan",
            execute_callback=self._execute,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=self._on_cancel,
            callback_group=group,
        )
        
    def _on_cancel(self, goal_handle):
        with self.lock:
            self.core.cancel()
        return CancelResponse.ACCEPT
    
    def _execute(self, goal_handle):
        goal = goal_handle.request
        with self.lock:
            pose = self._pose.get(self.now())
            position = pose[0] if pose is not None else None
            error = self.core.start(goal.plan_json, goal.plan_hash, self._control, position)
        if error is not None:
            goal_handle.abort()
            return ExecutePlan.Result(success=False, message=error)
        
        while rclpy.ok():
            with self.lock:
                now = self.now()
                pose = self._pose.get(now)
                position, yaw = pose if pose is not None else (None, 0.0)
                out = self.core.tick(position, yaw, self._control, now)
            self.publish_velocity(out.velocity)
            goal_handle.publish_feedback(ExecutePlan.Feedback(step=out.step, state = out.state))
            if out.finished:
                break
            time.sleep(TICK_PERIOD_S)
        self.publish_velocity(HOLD)                   # one explicit zero, then silence
        result = ExecutePlan.Result(success=out.state == "DONE", message=out.message or "")
        if result.success:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result
        
    def now(self) -> float:
        return time.monotonic()
    
    def _on_position(self, msg):
        pose = to_pose(msg)
        if pose is not None:
            with self.lock:
                self._pose.update(pose, self.now())
                
    def _on_control(self, msg):
        with self.lock:
            self._control = Control(msg.source, msg.armed)
    
    def _on_pause(self, request, response):
        with self.lock:
            response.success = self.core.busy
            if response.success:
                self.core.pause(self.now())
        response.message = "resumed" if response.success else "no plan is running"
        return response
    
    def _on_resume(self, request, response):
        with self.lock:
            response.success = self.core.busy
            if response.success:
                self.core.resume(self.now())
        response.message = "resumed" if response.success else "no plan is running"
        return response
    
    def publish_velocity(self, velocity):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.linear.z = velocity
        self._velocity_pub.publish(msg)
        
        
        
def main():
    rclpy.init()
    node = ExecutorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
