import os
from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, _ = main_with_errors(argv=[])
    assert rc == 0, 'Found code style errors / warnings'