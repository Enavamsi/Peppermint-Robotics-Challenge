# Copyright 2025 Your Organisation
# SPDX-License-Identifier: Apache-2.0

"""
test_visual_servoing.py
=======================
Unit tests for the color_nav VisualServoing node.

Run with:
    colcon test --packages-select color_nav
    colcon test-result --verbose
"""

import math
import pytest

import rclpy
from rclpy.parameter import Parameter

from color_nav.visual_servoing import VisualServoing, State


# ─────────────────────────────────────────────────────────────────────────────
# Fixtures
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture(scope="module", autouse=True)
def ros_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def node():
    n = VisualServoing()
    n.set_parameters([Parameter("show_debug_window", value=False)])
    yield n
    n.destroy_node()


# ─────────────────────────────────────────────────────────────────────────────
# Tests — FSM transitions
# ─────────────────────────────────────────────────────────────────────────────

class TestFSMTransitions:

    def test_search_when_no_target(self, node):
        twist = node._compute_velocity(cx_img=320, target_cx=None, target_cy=None)
        assert node.state == State.SEARCH
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.angular.z > 0.0

    def test_stop_overrides_tracking(self, node):
        node.front_dist = 0.5   # inside safe_distance
        twist = node._compute_velocity(cx_img=320, target_cx=320, target_cy=120)
        assert node.state == State.STOP
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.angular.z == pytest.approx(0.0)
        node.front_dist = float("inf")   # restore

    def test_align_when_error_large(self, node):
        node.front_dist = float("inf")
        # target 100 px off-centre → error_x = 100 > threshold=40
        twist = node._compute_velocity(cx_img=320, target_cx=420, target_cy=120)
        assert node.state == State.ALIGN
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.angular.z != pytest.approx(0.0)

    def test_approach_when_aligned(self, node):
        node.front_dist = float("inf")
        # target exactly at centre → error_x = 0
        twist = node._compute_velocity(cx_img=320, target_cx=320, target_cy=120)
        assert node.state == State.APPROACH
        assert twist.linear.x > 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Tests — P-controller
# ─────────────────────────────────────────────────────────────────────────────

class TestPController:

    def test_angular_clamped_positive(self, node):
        node.front_dist = float("inf")
        # Massive left error → clamp to +1.0
        twist = node._compute_velocity(cx_img=320, target_cx=320 + 10000, target_cy=0)
        assert twist.angular.z <= 1.0

    def test_angular_clamped_negative(self, node):
        node.front_dist = float("inf")
        twist = node._compute_velocity(cx_img=320, target_cx=320 - 10000, target_cy=0)
        assert twist.angular.z >= -1.0

    def test_angular_proportional(self, node):
        node.front_dist = float("inf")
        kp = node._p("kp")
        error = 100
        twist = node._compute_velocity(cx_img=320, target_cx=320 - error, target_cy=0)
        expected = kp * error
        assert twist.angular.z == pytest.approx(expected, abs=1e-4)
