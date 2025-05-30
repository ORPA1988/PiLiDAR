import pytest

pymeshlab = pytest.importorskip("pymeshlab")


def test_pymeshlab_has_version():
    assert hasattr(pymeshlab, "__version__")
