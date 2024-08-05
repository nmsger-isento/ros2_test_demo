# test_publisher.py
import pytest
from unittest.mock import MagicMock, patch
from demo.levensthein import LevenshteinService
from demo.service import (
    Service,
)
from interfaces.srv import LevenstheinSrv
from std_msgs.msg import String
import rclpy


# Fixtures initialize test functions for reliably, consistent, repeatable results
# Multiple fixtures are allowed, they can be nested, etc.
# https://docs.pytest.org/en/stable/how-to/fixtures.html#running-multiple-assert-statements-safely
@pytest.fixture
def service_node():
    # Initialize the `Publisher` node with a mocked LevenshteinService
    rclpy.init()
    levensthein_mock = MagicMock(LevenshteinService)
    node = Service(levensthein_mock)
    yield node
    node.destroy_node()
    rclpy.shutdown()


# Test that the node was correctly initialized, including its dependencies
def test_publisher_initialization(service_node):
    assert service_node.levensthein_service is not None
    assert service_node.srv is not None


@pytest.fixture(params=[1, 2, 3, 4, 5, 6, 7])
def get_levensthein_mock_distances(request):
    return request.param


# test_timer_callback will be run for each param in the get_levensthein_mock_distances fixture
# This allows easy/fast testing of different parameter settings, such as output values in this case
def test_get_levensthein_distance_callback(
    service_node, get_levensthein_mock_distances
):
    # Test that different output values are accepted and used
    service_node.levensthein_service.calculate.return_value = (
        get_levensthein_mock_distances
    )

    request = LevenstheinSrv.Request(word1="a", word2="b")
    response = LevenstheinSrv.Response()

    service_node.get_levensthein_distance_callback(request, response)

    # Assert that LevenshteinService's calculate method was called with correct input values
    service_node.levensthein_service.calculate.assert_called_once_with(
        request.word1, request.word2
    )

    # Assert that the response contains the correct value
    assert response.distance == get_levensthein_mock_distances


# should throw exception if request type is incorrect
def test_get_levensthein_distance_callback_wrong_input(service_node):
    service_node.levensthein_service.calculate.return_value = 0
    response = LevenstheinSrv.Response()

    # The service should throw an exception if the input parameters use the incorrect type
    request = LevenstheinSrv.Request(word1="a", word2={"TEST": "TEST"})
    with pytest.raises(Exception):
        service_node.get_levensthein_distance_callback(request, response)

    request = LevenstheinSrv.Request(word1={"TEST": "TEST"}, word2="a")
    with pytest.raises(Exception):
        service_node.get_levensthein_distance_callback(request, response)
