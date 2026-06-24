"""Small geometry helpers shared by probing scripts."""
import math


def normalize(vector, label="vector"):
    """Return a unit-length copy of a 3D vector, raising ValueError on zero input."""
    length = math.sqrt(sum(v * v for v in vector))
    if length <= 1e-12:
        raise ValueError(f"{label} cannot be zero")
    return [v / length for v in vector]


def add(a, b):
    """Return the element-wise sum of two equal-length vectors."""
    return [x + y for x, y in zip(a, b)]


def scale(vector, value):
    """Return a vector scaled by a scalar value (element-wise multiplication)."""
    return [v * value for v in vector]
