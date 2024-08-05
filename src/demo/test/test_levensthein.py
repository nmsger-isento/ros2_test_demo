import pytest
from demo.levensthein import LevenshteinService


@pytest.fixture
def levenshtein_service():
    return LevenshteinService()


def test_identical_strings(levenshtein_service):
    assert levenshtein_service.calculate("test", "test") == 0


def test_empty_strings(levenshtein_service):
    assert levenshtein_service.calculate("", "") == 0


def test_one_empty_string(levenshtein_service):
    assert levenshtein_service.calculate("abc", "") == 3
    assert levenshtein_service.calculate("", "abc") == 3


def test_different_strings(levenshtein_service):
    assert levenshtein_service.calculate("kitten", "sitting") == 3
    assert levenshtein_service.calculate("flaw", "lawn") == 2
    assert levenshtein_service.calculate("gumbo", "gambol") == 2


def test_case_sensitivity(levenshtein_service):
    assert levenshtein_service.calculate("Kitten", "kitten") == 1
