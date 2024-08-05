# test_publisher.py
import pytest
from unittest.mock import MagicMock, patch
from demo.publisher import (
    Publisher,
)
from std_msgs.msg import String
import rclpy


# Fixtures initialize test functions for reliably, consistent, repeatable results
# Multiple fixtures are allowed, they can be nested, etc.
# https://docs.pytest.org/en/stable/how-to/fixtures.html#running-multiple-assert-statements-safely
@pytest.fixture
def publisher_node():
    # Create a publisher node using a mocked publisher
    rclpy.init()
    with patch("demo.publisher.Node.create_publisher") as mock_create_publisher:
        mock_publisher = MagicMock()
        mock_create_publisher.return_value = mock_publisher
        node = Publisher()
        yield node, mock_publisher
        node.destroy_node()
    rclpy.shutdown()


# Test that the node was correctly initialized, including its dependencies
def test_publisher_initialization(publisher_node):
    node, _ = publisher_node
    assert node.publisher_ is not None
    assert node.timer is not None


def test_timer_callback(publisher_node):
    node, mock_publisher = publisher_node
    node.timer_callback()

    # Assert that publisher is publishing the correct message
    expected_message = String()
    expected_message.data = "Hello World"
    mock_publisher.publish.assert_called_with(expected_message)
