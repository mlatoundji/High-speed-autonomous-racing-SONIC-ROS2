import math as m


def normalise_angle(angle):
    return m.atan2(m.sin(angle), m.cos(angle))
