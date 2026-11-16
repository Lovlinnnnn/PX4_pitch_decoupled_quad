#!/usr/bin/env python3
"""Write the pitch-decoupled quad SDF with the 22012 flight tilt envelope."""

import argparse
import math
import sys
import xml.etree.ElementTree as ET


def configure_flight_limits(root):
    model = root.find("model[@name='pitch_decoupled_quad']")
    if model is None:
        raise ValueError("expected the pitch_decoupled_quad model")

    servo = model.find("plugin[@filename='libgazebo_tilt_servo_plugin.so']")
    if servo is None or servo.find("maxAngleRad") is None:
        raise ValueError("expected the pitch_decoupled_quad tilt servo configuration")

    # At 45 degrees of body pitch, lateral acceleration can add atan(0.14) to the tilt demand.
    angle = math.radians(55.0)
    joint_names = (servo.findtext("frontJoint"), servo.findtext("rearJoint"))
    if not all(joint_names) or joint_names[0] == joint_names[1]:
        raise ValueError("expected distinct front and rear tilt joints")

    for name in joint_names:
        joint = model.find("joint[@name='{}']".format(name))
        limits = joint.find("axis/limit") if joint is not None and joint.get("type") == "revolute" else None
        if limits is None or limits.find("lower") is None or limits.find("upper") is None:
            raise ValueError("expected revolute limits on " + name)
        limits.find("lower").text = str(-angle)
        limits.find("upper").text = str(angle)

    servo.find("maxAngleRad").text = str(angle)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sdf")
    args = parser.parse_args()
    root = ET.parse(args.sdf).getroot()
    configure_flight_limits(root)
    sys.stdout.write(ET.tostring(root, encoding="unicode"))
