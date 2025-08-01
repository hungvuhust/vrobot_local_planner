"""
Service client utility for ROS2 rclpy.
"""
import time
from typing import TypeVar, Generic, Optional, Type, Any
import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_services_default
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future

ServiceT = TypeVar('ServiceT')
RequestT = TypeVar('RequestT')
ResponseT = TypeVar('ResponseT')


class ServiceClient(Generic[ServiceT]):
    """
    A simple service client that can be used to call services.

    Args:
        service_type: The type of the service to call
        service_name: Name of the service to call
        node: Node to create the service client off of
    """

    def __init__(self, service_type: Type[ServiceT], service_name: str, node: Node, max_retries: int = 3):
        """
        Constructor for ServiceClient.

        Args:
            service_type: The service type class
            service_name: Name of the service to call
            node: Node to create the service client off of
        """
        self.service_name = service_name
        self.node = node
        self.service_type = service_type

        # Create callback group for exclusive execution
        self.callback_group = MutuallyExclusiveCallbackGroup()

        # Create executor for the callback group
        self.callback_group_executor = SingleThreadedExecutor()
        self.callback_group_executor.add_node(node)

        # Create the service client
        self.client = node.create_client(
            service_type,
            service_name,
            qos_profile=qos_profile_services_default,
            callback_group=self.callback_group
        )

        self.max_retries = max_retries

    def invoke(self, request: Any, timeout_sec: Optional[float] = None) -> Optional[Any]:
        """
        Invoke the service and block until completed or timed out.

        Args:
            request: The request object to call the service using
            timeout_sec: Maximum timeout to wait for in seconds, None for infinite

        Returns:
            Response object from the service call, or None if failed
        """
        max_retries = self.max_retries

        for i in range(max_retries):
            try:
                # Wait for service to be available
                while not self.client.wait_for_service(timeout_sec=1.0):
                    if not rclpy.ok():
                        raise RuntimeError(
                            f"{self.service_name} service client: interrupted while waiting for service"
                        )
                    self.node.get_logger().info(
                        f"{self.service_name} service client: waiting for service to appear..."
                    )

                self.node.get_logger().debug(
                    f"{self.service_name} service client: send async request"
                )

                # Send async request
                future = self.client.call_async(request)

                # Wait for response with timeout
                if timeout_sec is not None:
                    start_time = time.time()
                    while not future.done():
                        rclpy.spin_once(self.node, timeout_sec=0.1)
                        if time.time() - start_time > timeout_sec:
                            raise RuntimeError(
                                f"{self.service_name} service client: async_send_request timed out"
                            )
                        if not rclpy.ok():
                            raise RuntimeError(
                                f"{self.service_name} service client: interrupted during request"
                            )
                else:
                    # Wait indefinitely
                    while not future.done():
                        rclpy.spin_once(self.node, timeout_sec=0.1)
                        if not rclpy.ok():
                            raise RuntimeError(
                                f"{self.service_name} service client: interrupted during request"
                            )

                return future.result()

            except Exception as e:
                self.node.get_logger().error(
                    f"{self.service_name} service client: {str(e)}, retrying ({i + 1}/{max_retries})"
                )
                if i == max_retries - 1:
                    return None  # Return None if all retries failed
                # Wait a bit before retrying
                time.sleep(0.1)

        return None

    def invoke_with_response(self, request: Any) -> tuple[bool, Optional[Any]]:
        """
        Invoke the service and block until completed.

        Args:
            request: The request object to call the service using

        Returns:
            Tuple of (success: bool, response: Optional[Any])
        """
        max_retries = self.max_retries

        for i in range(max_retries):
            try:
                # Wait for service to be available
                while not self.client.wait_for_service(timeout_sec=1.0):
                    if not rclpy.ok():
                        raise RuntimeError(
                            f"{self.service_name} service client: interrupted while waiting for service"
                        )
                    self.node.get_logger().info(
                        f"{self.service_name} service client: waiting for service to appear..."
                    )

                self.node.get_logger().debug(
                    f"{self.service_name} service client: send async request"
                )

                # Send async request
                future = self.client.call_async(request)

                # Wait for response
                while not future.done():
                    rclpy.spin_once(self.node, timeout_sec=0.1)
                    if not rclpy.ok():
                        return False, None

                response = future.result()
                return True, response

            except Exception as e:
                self.node.get_logger().error(
                    f"{self.service_name} service client: {str(e)}, retrying ({i + 1}/{max_retries})"
                )
                if i == max_retries - 1:
                    return False, None  # Return False if all retries failed
                # Wait a bit before retrying
                time.sleep(0.1)

        return False, None

    def wait_for_service(self, timeout_sec: Optional[float] = None) -> bool:
        """
        Block until a service is available or timeout.

        Args:
            timeout_sec: Maximum timeout to wait for in seconds, None for infinite

        Returns:
            True if service is available, False otherwise
        """
        return self.client.wait_for_service(timeout_sec=timeout_sec)

    def get_service_name(self) -> str:
        """
        Get the service name.

        Returns:
            Service name string
        """
        return self.service_name
