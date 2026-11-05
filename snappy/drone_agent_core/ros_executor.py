import threading
import time

import rclpy
from px4_agent_interfaces.action import ExecutePlan
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from drone_agent_core.robot import ExecutionStatus

FIRST_ANSWER_S = 1.5       # how long execute() waits for the executor's first word
SERVICE_WAIT_S = 2.0
NOT_RUNNING = "The executor node is not running (python3 -m drone_agent_core.executor_node)."


class RosExecutor:
    """Operator side of the executor: sends the approved plan and the pause/resume/stop commands.

    Only ever called by chat commands, never by the model.
    """

    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self._lock = threading.Lock()
        self._status = ExecutionStatus("IDLE")
        self._goal_handle = None
        self._got_feedback = False
        self._node = Node("snappy_operator")
        self._action = ActionClient(self._node, ExecutePlan, "execute_plan")
        self._pause = self._node.create_client(Trigger, "/executor/pause")
        self._resume = self._node.create_client(Trigger, "/executor/resume")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        threading.Thread(target=self._executor.spin, daemon=True).start()

    @staticmethod
    def _wait(future, timeout_s: float):
        """The future's result, or None if it does not arrive in time (called off the spin thread)"""
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        return future.result() if done.wait(timeout_s) else None

    def _on_feedback(self, message):
        feedback = message.feedback
        with self._lock:
            self._got_feedback = True
            self._status = ExecutionStatus(feedback.state, feedback.step)

    def _on_result(self, future):
        wrapped = future.result()
        result = wrapped.result
        with self._lock:
            state = "DONE" if result.success else "ABORTED"
            self._status = ExecutionStatus(state, self._status.step, result.message or None)

    def execute(self, plan_json: str, plan_hash: str) -> str | None:
        if not self._action.wait_for_server(timeout_sec=1.0):
            return NOT_RUNNING
        with self._lock:
            self._got_feedback = False
        goal = ExecutePlan.Goal(plan_json=plan_json, plan_hash=plan_hash)
        handle = self._wait(self._action.send_goal_async(goal, feedback_callback=self._on_feedback),
                            FIRST_ANSWER_S)
        if handle is None:
            return "the executor did not answer"
        if not handle.accepted:
            return "the executor did not accept the goal"
        with self._lock:
            self._goal_handle = handle
            self._status = ExecutionStatus("RUNNING")
        handle.get_result_async().add_done_callback(self._on_result)

        deadline = time.monotonic() + FIRST_ANSWER_S
        while time.monotonic() < deadline:
            with self._lock:
                status, got_feedback = self._status, self._got_feedback
            if status.state in ("ABORTED", "DONE"):         # refused (or finished) right away
                return status.message or "the executor ended the plan immediately"
            if got_feedback:
                return None
            time.sleep(0.05)
        return None

    def _call(self, client, what: str) -> str:
        if not client.wait_for_service(timeout_sec=1.0):
            return NOT_RUNNING
        response = self._wait(client.call_async(Trigger.Request()), SERVICE_WAIT_S)
        if response is None:
            return f"No answer to {what}."
        return response.message[:1].upper() + response.message[1:] + "."

    def pause(self) -> str:
        return self._call(self._pause, "pause")

    def resume(self) -> str:
        return self._call(self._resume, "resume")

    def stop(self) -> str:
        with self._lock:
            handle, state = self._goal_handle, self._status.state
        if handle is None or state not in ("RUNNING", "PAUSED"):
            return "No plan is running."
        self._wait(handle.cancel_goal_async(), SERVICE_WAIT_S)
        return "Stopping: holding position."

    def status(self) -> ExecutionStatus:
        with self._lock:
            return self._status

    def close(self):
        self._executor.shutdown()
        self._node.destroy_node()
