"""a handful of utility functions used by GlitterPOS."""
import math
import time

def timestamp(val):
    """print a human-readable timestamp"""
    timestamp = val
    return '{}/{}/{} {:02}:{:02}:{:02}'.format(
        timestamp.tm_year,
        timestamp.tm_mon,
        timestamp.tm_mday,
        timestamp.tm_hour,
        timestamp.tm_min,
        timestamp.tm_sec
    )
