#!/usr/bin/env bash
set -e

python3 robot/robot.py test
# E501 line too long
pycodestyle robot/ --ignore=E501
