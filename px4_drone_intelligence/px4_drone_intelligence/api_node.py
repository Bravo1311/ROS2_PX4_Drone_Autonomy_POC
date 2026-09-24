import os
import rclpy
from rclpy.node import Node

from px4_drone_intelligence.api_client import ask, resolve_key, DEFAULT_MODELS, API_MAP

class ApiNode(Node):
    def __init__(self):
        super().__init__('api_node')

        self.declare_parameter('provider', 'openRouter')
        self.declare_parameter('prompt', '')
        self.declare_parameter('image_path', '')
        self.declare_parameter('model', '')
        self.declare_parameter('max_tokens', 512)
        self.declare_parameter('timeout', 60.0)

        provider = self.get_parameter('provider').value
        prompt = self.get_parameter('prompt').value
        image_path = self.get_parameter('image_path').value
        model = self.get_parameter('model').value or None
        max_tokens = self.get_parameter('max_tokens').value
        timeout = self.get_parameter('timeout').value

        if provider not in API_MAP:
            self.get_logger().fatal(
                f"Unknown provider '{provider}'. Expected {sorted(API_MAP)}"
            )
            raise SystemExit(2)

        if not prompt:
            self.get_logger().fatal(
                f"No prompt. Pass one with --ros-args -p prompt:='your question'"
            )
            raise SystemExit(2)

        try:
            key = resolve_key(provider)
        except ValueError as e:
            self.get_logger().fatal(str(e))
            raise SystemExit(2)

        self.get_logger().info(
            f"provider={provider} model={model or DEFAULT_MODELS[provider]} "
            f"key=...{key[-4:]}"
        )

        if image_path:
            self.get_logger().info(f"image={image_path}")
        self.get_logger().info(f"prompt={prompt!r}")

        try: 
            text, meta = ask(
                provider, prompt, image_path=image_path or None,
                key=key, model=model, max_tokens=max_tokens, timeout=timeout
            )
        except ValueError as e:          # bad image path / type / size
            self.get_logger().fatal(str(e))
            raise SystemExit(2)
        except RuntimeError as e:        # HTTP, network, refusal
            self.get_logger().error(str(e))
            raise SystemExit(1)

        if meta['finish_reason'] != 'stop':
            self.get_logger().warn(
                f"Response truncated (finish_reason={meta['finish_reason']});"
                f"raise max_tokens if the answer looks cut off.")
        
        self.get_logger().info(f"--- response ---\n{text.strip()}")
        self.get_logger().info(
            f"tokens={meta['total_tokens']} cost=${meta['cost']}"
            if meta['cost'] is not None else f"tokens={meta['total_tokens']}")

        raise SystemExit(0)

def main():
    rclpy.init()
    node = None
    try:
        node = ApiNode()        
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()