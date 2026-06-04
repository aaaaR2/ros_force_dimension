# Copyright 2026 Carnegie Mellon University Neuromechatronics Lab
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.
"""Reference spec for src/force_dimension/src/ellipse_helpers.hpp::ellipse_radial_force - keep in sync (plan 80-02).

This module contains:

- ``ellipse_radial_force``: pure-Python reference implementing the
  implicit-ellipse gradient formula from 80-RESEARCH.md Q2.
- ``rim_point``: parametric ellipse point helper from 80-RESEARCH.md Q3.
- Six unit tests verifying circle reduction, zero-on-rim, depth sign,
  deadzone skip, unit normal, and rim placement.

These tests are GREEN against the reference (the reference is correct by
construction).  They serve as the cross-check signal for the C++ port in
plan 80-02, which cannot be unit-tested directly (no C++ gtest infra in this
repo per RESEARCH Q5).
"""
from __future__ import annotations

import math

import pytest

pytestmark = pytest.mark.unit

# ---------------------------------------------------------------------------
# Pure-Python reference implementation
# ---------------------------------------------------------------------------


def ellipse_radial_force(
    d0: float,
    d1: float,
    vel0: float,
    vel1: float,
    sa: float,
    sb: float,
    Kp: float,
    Kv: float,
    fmax: float,
    center_deadzone: float,
) -> tuple[float, float, float, float, float]:
    """Compute the bilateral ellipse radial spring-damper force.

    Implements the implicit-ellipse gradient formula from 80-RESEARCH.md Q2.
    All lengths in metres; forces in Newtons.

    Parameters
    ----------
    d0:
        Displacement along the first in-plane axis from the ellipse centre (m).
    d1:
        Displacement along the second in-plane axis from the ellipse centre (m).
    vel0:
        Velocity component along the first in-plane axis (m/s).
    vel1:
        Velocity component along the second in-plane axis (m/s).
    sa:
        Semi-axis along the first in-plane axis (m).  Must be > 0.
    sb:
        Semi-axis along the second in-plane axis (m).  Must be > 0.
    Kp:
        Radial spring stiffness (N/m).
    Kv:
        Radial damping coefficient (N·s/m).
    fmax:
        Force clamp magnitude (N).  Output forces are in [−fmax, +fmax].
    center_deadzone:
        Absolute deadzone radius (m).  When the normalised radius
        ``r_eff <= center_deadzone / min(sa, sb)`` the function returns zeros.

    Returns
    -------
    force0:
        Force along the first in-plane axis (N).
    force1:
        Force along the second in-plane axis (N).
    n0:
        Unit outward normal component along the first in-plane axis.
    n1:
        Unit outward normal component along the second in-plane axis.
    depth:
        Signed penetration depth (m). Negative inside the ellipse, positive
        outside, zero on the rim.  Returns 0.0 when inside the deadzone.
    """
    # Normalised coordinates — unit circle in "ellipse space".
    u0 = d0 / sa
    u1 = d1 / sb
    r_eff = math.sqrt(u0 * u0 + u1 * u1)  # 1.0 exactly on the ellipse

    # Deadzone skip: treat the region near the centre as a singularity-free zone.
    deadzone_norm = center_deadzone / min(sa, sb)
    if r_eff <= deadzone_norm:
        return 0.0, 0.0, 0.0, 0.0, 0.0

    # Outward unit normal via the implicit-ellipse gradient:
    #   grad f = (2*d0/sa^2, 2*d1/sb^2)  ∝  (u0/sa, u1/sb)
    gx = u0 / sa   # = d0 / sa^2
    gy = u1 / sb   # = d1 / sb^2
    g_mag = math.sqrt(gx * gx + gy * gy)
    n0 = gx / g_mag   # unit outward normal, first axis
    n1 = gy / g_mag   # unit outward normal, second axis

    # Radial velocity (projection of velocity onto the outward normal).
    v_r = vel0 * n0 + vel1 * n1

    # Signed depth in metres.  Exact (r − R) when sa == sb == R.
    depth = (r_eff - 1.0) * math.sqrt(sa * sb)

    # Bilateral radial spring-damper, clamped to ±fmax.
    raw_fr = -Kp * depth - Kv * v_r
    f_r = max(-fmax, min(fmax, raw_fr))

    force0 = f_r * n0
    force1 = f_r * n1
    return force0, force1, n0, n1, depth


def rim_point(
    c0: float,
    c1: float,
    sa: float,
    sb: float,
    theta: float,
) -> tuple[float, float]:
    """Return the Cartesian coordinates of a point on the ellipse rim.

    Parameters
    ----------
    c0:
        Ellipse centre coordinate on the first in-plane axis (m).
    c1:
        Ellipse centre coordinate on the second in-plane axis (m).
    sa:
        Semi-axis along the first in-plane axis (m).
    sb:
        Semi-axis along the second in-plane axis (m).
    theta:
        Parametric angle (radians).

    Returns
    -------
    (x0, x1):
        Point on the ellipse: ``(c0 + sa*cos(theta), c1 + sb*sin(theta))``.
    """
    return (c0 + sa * math.cos(theta), c1 + sb * math.sin(theta))


# ---------------------------------------------------------------------------
# Unit tests
# ---------------------------------------------------------------------------


class TestCircleReduction:
    """When sa == sb == R, the ellipse formula must exactly equal the circle formula."""

    def test_circle_reduction(self) -> None:
        """Ellipse force EXACTLY equals legacy circle clamp for several off-centre points."""
        R = 0.04
        Kp = 500.0
        Kv = 10.0
        fmax = 20.0
        center_deadzone = 0.005

        # Legacy circle formula (from constraints.cpp lines 151–184):
        #   f_r = clamp(-Kp * (r - R) - Kv * v_r, -fmax, fmax)
        #   n0 = d0 / r,  n1 = d1 / r
        def circle_force(d0: float, d1: float, v0: float, v1: float) -> tuple[float, float]:
            r = math.sqrt(d0 * d0 + d1 * d1)
            if r <= center_deadzone:
                return 0.0, 0.0
            n0 = d0 / r
            n1 = d1 / r
            v_r = v0 * n0 + v1 * n1
            raw = -Kp * (r - R) - Kv * v_r
            f_r = max(-fmax, min(fmax, raw))
            return f_r * n0, f_r * n1

        # Test several off-centre points (both inside and outside the rim).
        test_points = [
            (0.05, 0.0, 0.0, 0.0),    # outside, no velocity
            (-0.03, 0.0, 0.1, 0.0),   # inside, radial velocity
            (0.0, 0.06, 0.0, -0.2),   # outside, inward velocity
            (0.03, 0.03, 0.05, 0.05), # diagonal outside
            (0.01, -0.01, 0.0, 0.0),  # inside, near centre
        ]

        for d0, d1, v0, v1 in test_points:
            f_ellipse_0, f_ellipse_1, _, _, _ = ellipse_radial_force(
                d0, d1, v0, v1, R, R, Kp, Kv, fmax, center_deadzone
            )
            f_circle_0, f_circle_1 = circle_force(d0, d1, v0, v1)
            assert math.isclose(f_ellipse_0, f_circle_0, abs_tol=1e-9), (
                f"force0 mismatch at ({d0},{d1}): ellipse={f_ellipse_0}, circle={f_circle_0}"
            )
            assert math.isclose(f_ellipse_1, f_circle_1, abs_tol=1e-9), (
                f"force1 mismatch at ({d0},{d1}): ellipse={f_ellipse_1}, circle={f_circle_1}"
            )


class TestZeroOnRim:
    """A point exactly on the ellipse rim with zero velocity yields zero radial force."""

    def test_zero_on_rim(self) -> None:
        """Point on ellipse at parametric angle theta, zero velocity -> |force| < 1e-9."""
        sa = 0.05
        sb = 0.03
        Kp = 400.0
        Kv = 8.0
        fmax = 20.0
        center_deadzone = 0.001

        for theta in [0.0, math.pi / 6, math.pi / 3, math.pi / 2, math.pi, 3 * math.pi / 2]:
            d0, d1 = rim_point(0.0, 0.0, sa, sb, theta)
            f0, f1, _, _, depth = ellipse_radial_force(
                d0, d1, 0.0, 0.0, sa, sb, Kp, Kv, fmax, center_deadzone
            )
            f_mag = math.sqrt(f0 * f0 + f1 * f1)
            assert f_mag < 1e-9, (
                f"Non-zero force {f_mag} at rim theta={theta:.3f}: depth={depth}"
            )


class TestDepthSign:
    """Inside the rim gives outward restoring force; outside gives inward restoring force."""

    def test_depth_sign(self) -> None:
        """sign(f_r) == -sign(depth) at several interior and exterior points."""
        sa = 0.05
        sb = 0.03
        Kp = 500.0
        Kv = 0.0   # no velocity term — isolates spring sign
        fmax = 100.0
        center_deadzone = 0.001

        # Interior point: depth < 0, expect f_r > 0 (outward push toward rim).
        d0_in, d1_in = 0.02, 0.01   # well inside ellipse
        f0, f1, n0, n1, depth = ellipse_radial_force(
            d0_in, d1_in, 0.0, 0.0, sa, sb, Kp, Kv, fmax, center_deadzone
        )
        assert depth < 0, f"Expected depth < 0 for interior point, got {depth}"
        f_r = f0 * n0 + f1 * n1  # project back onto normal
        assert f_r > 0, f"Expected f_r > 0 (outward) for interior point, got f_r={f_r}"

        # Exterior point: depth > 0, expect f_r < 0 (inward pull toward rim).
        d0_out, d1_out = 0.07, 0.0  # well outside ellipse (sa=0.05)
        f0, f1, n0, n1, depth = ellipse_radial_force(
            d0_out, d1_out, 0.0, 0.0, sa, sb, Kp, Kv, fmax, center_deadzone
        )
        assert depth > 0, f"Expected depth > 0 for exterior point, got {depth}"
        f_r = f0 * n0 + f1 * n1
        assert f_r < 0, f"Expected f_r < 0 (inward) for exterior point, got f_r={f_r}"


class TestDeadzoneSkip:
    """Points at or inside the deadzone radius yield exactly (0, 0)."""

    def test_deadzone_skip(self) -> None:
        """r_eff <= center_deadzone / min(sa, sb) => (force0, force1) == (0.0, 0.0)."""
        sa = 0.05
        sb = 0.03
        Kp = 500.0
        Kv = 10.0
        fmax = 20.0
        center_deadzone = 0.01

        # Boundary: r_eff == deadzone_norm (still inside deadzone).
        deadzone_norm = center_deadzone / min(sa, sb)

        # Point exactly AT the deadzone boundary in normalised space.
        # u0 = deadzone_norm, u1 = 0  =>  d0 = deadzone_norm * sa, d1 = 0.
        d0 = deadzone_norm * sa
        d1 = 0.0
        f0, f1, n0, n1, depth = ellipse_radial_force(
            d0, d1, 0.5, 0.5, sa, sb, Kp, Kv, fmax, center_deadzone
        )
        assert (f0, f1) == (0.0, 0.0), f"Expected (0,0) at deadzone boundary, got ({f0},{f1})"

        # Point strictly inside the deadzone.
        d0 = deadzone_norm * sa * 0.5
        d1 = 0.0
        f0, f1, _, _, _ = ellipse_radial_force(
            d0, d1, 1.0, 1.0, sa, sb, Kp, Kv, fmax, center_deadzone
        )
        assert (f0, f1) == (0.0, 0.0), f"Expected (0,0) inside deadzone, got ({f0},{f1})"

        # Origin itself.
        f0, f1, _, _, _ = ellipse_radial_force(
            0.0, 0.0, 0.0, 0.0, sa, sb, Kp, Kv, fmax, center_deadzone
        )
        assert (f0, f1) == (0.0, 0.0), "Expected (0,0) at origin"


class TestNormalIsUnit:
    """The returned (n0, n1) normal vector has unit magnitude at an arbitrary point."""

    def test_normal_is_unit(self) -> None:
        """|(n0, n1)| == 1.0 (atol 1e-12) at various off-centre points outside deadzone."""
        sa = 0.06
        sb = 0.04
        Kp = 300.0
        Kv = 5.0
        fmax = 25.0
        center_deadzone = 0.002

        test_points = [
            (0.08, 0.0),
            (0.0, 0.06),
            (0.05, 0.03),
            (-0.04, 0.05),
            (0.03, -0.035),
        ]
        for d0, d1 in test_points:
            _, _, n0, n1, _ = ellipse_radial_force(
                d0, d1, 0.0, 0.0, sa, sb, Kp, Kv, fmax, center_deadzone
            )
            mag = math.sqrt(n0 * n0 + n1 * n1)
            assert math.isclose(mag, 1.0, abs_tol=1e-12), (
                f"Normal magnitude {mag} != 1.0 at ({d0},{d1})"
            )


class TestRimPlacement:
    """rim_point returns the correct parametric ellipse coordinate."""

    def test_rim_placement(self) -> None:
        """rim_point(0, 0, 0.05, 0.03, pi/2) == (0, 0.03) within 1e-9."""
        x0, x1 = rim_point(0.0, 0.0, 0.05, 0.03, math.pi / 2)
        assert math.isclose(x0, 0.0, abs_tol=1e-9), f"x0={x0}, expected 0.0"
        assert math.isclose(x1, 0.03, abs_tol=1e-9), f"x1={x1}, expected 0.03"

    def test_rim_placement_theta_zero(self) -> None:
        """rim_point(c0, c1, sa, sb, 0) == (c0 + sa, c1)."""
        c0, c1, sa, sb = 0.01, -0.02, 0.05, 0.03
        x0, x1 = rim_point(c0, c1, sa, sb, 0.0)
        assert math.isclose(x0, c0 + sa, abs_tol=1e-12)
        assert math.isclose(x1, c1, abs_tol=1e-12)

    def test_rim_placement_theta_pi(self) -> None:
        """rim_point(c0, c1, sa, sb, pi) == (c0 - sa, c1)."""
        c0, c1, sa, sb = 0.0, 0.0, 0.05, 0.03
        x0, x1 = rim_point(c0, c1, sa, sb, math.pi)
        assert math.isclose(x0, -sa, abs_tol=1e-9)
        assert math.isclose(x1, 0.0, abs_tol=1e-9)
