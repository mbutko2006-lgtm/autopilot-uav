#!/usr/bin/env python3
import collections
import collections.abc

# Patch for Python 3.10+ compatibility with dronekit
if not hasattr(collections, "MutableMapping"):
    collections.MutableMapping = collections.abc.MutableMapping

import argparse
import math
import time
from dataclasses import dataclass, field

from dronekit import VehicleMode, connect
from pymavlink import mavutil


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    i_min: float = -1e9
    i_max: float = 1e9
    out_min: float = -1e9
    out_max: float = 1e9
    integral: float = 0.0
    prev_error: float = 0.0
    initialized: bool = False
    last_p: float = 0.0
    last_i: float = 0.0
    last_d: float = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.initialized = False
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            dt = 1e-3

        if not self.initialized:
            self.prev_error = error
            self.initialized = True

        self.integral += error * dt
        self.integral = clamp(self.integral, self.i_min, self.i_max)

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        self.last_p = self.kp * error
        self.last_i = self.ki * self.integral
        self.last_d = self.kd * derivative
        out = self.last_p + self.last_i + self.last_d
        return clamp(out, self.out_min, self.out_max)


@dataclass
class FlightConfig:
    connect_str: str = "127.0.0.1:14550"
    target_lat: float = 50.443326
    target_lon: float = 30.448078
    target_alt_m: float = 300.0

    arrival_radius_m: float = 25.0
    slow_radius_m: float = 60.0
    final_radius_m: float = 22.0
    touchdown_radius_m: float = 5.0

    loop_dt_s: float = 0.2
    takeoff_timeout_s: float = 120.0
    cruise_timeout_s: float = 1200.0
    landing_timeout_s: float = 600.0

    wind_spd: float = 5.0
    wind_dir: float = 30.0
    wind_turb: float = 3.0
    wind_turb_freq: float = 0.25

    rc_slew_per_sec: float = 250.0
    alt_safety_margin_m: float = 30.0
    alt_safety_throttle: float = 1240.0

    alt_takeoff_pid: PID = field(default_factory=lambda: PID(
        kp=3.8, ki=0.03, kd=1.5,
        i_min=-60, i_max=60,
        out_min=-170, out_max=140
    ))

    # Менш агресивний набір висоти в cruise
    alt_cruise_pid: PID = field(default_factory=lambda: PID(
        kp=4.0, ki=0.16, kd=1.5,
        i_min=-70, i_max=70,
        out_min=-145, out_max=145
    ))

    cruise_x_pid: PID = field(default_factory=lambda: PID(
        kp=3.8, ki=0.0, kd=0.45,
        i_min=-25, i_max=25,
        out_min=-280, out_max=280
    ))

    cruise_y_pid: PID = field(default_factory=lambda: PID(
        kp=3.4, ki=0.0, kd=0.45,
        i_min=-25, i_max=25,
        out_min=-240, out_max=240
    ))

    align_x_pid: PID = field(default_factory=lambda: PID(
        kp=4.2, ki=0.0, kd=0.5,
        i_min=-18, i_max=18,
        out_min=-190, out_max=190
    ))

    align_y_pid: PID = field(default_factory=lambda: PID(
        kp=4.2, ki=0.0, kd=0.5,
        i_min=-18, i_max=18,
        out_min=-190, out_max=190
    ))

    # Сильніша фінальна XY-корекція
    land_x_pid: PID = field(default_factory=lambda: PID(
        kp=5.5, ki=0.13, kd=1.2,
        i_min=-12, i_max=12,
        out_min=-170, out_max=170
    ))

    land_y_pid: PID = field(default_factory=lambda: PID(
        kp=5.5, ki=0.13, kd=1.2,
        i_min=-12, i_max=12,
        out_min=-170, out_max=170
    ))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def apply_altitude_safety_cap(throttle: float, alt: float, cfg: FlightConfig) -> float:
    if alt > cfg.target_alt_m + cfg.alt_safety_margin_m:
        return min(throttle, cfg.alt_safety_throttle)
    return throttle


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)

    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return r * c


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)

    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    brng = math.degrees(math.atan2(y, x))
    return (brng + 360.0) % 360.0


def wrap_angle_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def relative_ne_m(lat: float, lon: float, target_lat: float, target_lon: float):
    r = 6378137.0
    dlat = math.radians(target_lat - lat)
    dlon = math.radians(target_lon - lon)
    north = dlat * r
    east = dlon * r * math.cos(math.radians((lat + target_lat) * 0.5))
    return north, east


class RCController:
    def __init__(self, vehicle, slew_per_sec: float = 250.0):
        self.vehicle = vehicle
        self.slew_per_sec = slew_per_sec
        self.current = {"1": 1500.0, "2": 1500.0, "3": 1000.0, "4": 1500.0}

    def reset_neutral(self, throttle=1000.0):
        self.current = {"1": 1500.0, "2": 1500.0, "3": float(throttle), "4": 1500.0}
        self._push()

    def send(self, roll=1500.0, pitch=1500.0, throttle=1500.0, yaw=1500.0, dt=0.2):
        desired = {
            "1": clamp(float(roll), 1000, 2000),
            "2": clamp(float(pitch), 1000, 2000),
            "3": clamp(float(throttle), 1000, 2000),
            "4": clamp(float(yaw), 1000, 2000),
        }

        max_step = self.slew_per_sec * max(dt, 0.01)
        for ch in self.current:
            delta = desired[ch] - self.current[ch]
            delta = clamp(delta, -max_step, max_step)
            self.current[ch] += delta

        self._push()

    def clear(self):
        self.vehicle.channels.overrides = {}

    def _push(self):
        self.vehicle.channels.overrides = {k: int(v) for k, v in self.current.items()}


def wait_for_gps(vehicle, timeout_s=60):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        loc = vehicle.location.global_relative_frame
        if loc and loc.lat is not None and loc.lon is not None:
            return
        time.sleep(0.5)
    raise TimeoutError("GPS data not available in time")


def configure_sim_wind(vehicle, cfg: FlightConfig):
    print(
        "Setting SIM wind: "
        f"SPD={cfg.wind_spd} DIR={cfg.wind_dir} TURB={cfg.wind_turb} TURB_FREQ={cfg.wind_turb_freq}"
    )
    for name, value in (
        ("SIM_WIND_SPD", cfg.wind_spd),
        ("SIM_WIND_DIR", cfg.wind_dir),
        ("SIM_WIND_TURB", cfg.wind_turb),
        ("SIM_WIND_TURB_FREQ", cfg.wind_turb_freq),
    ):
        if name not in vehicle.parameters:
            print(f"Wind parameter {name} is not supported by this firmware, skipping")
            continue
        try:
            vehicle.parameters[name] = value
        except Exception as error:
            print(f"Wind parameter {name} not applied: {error}")
    time.sleep(1.0)


def configure_sitl_vehicle(vehicle):
    print("Applying SITL arming/failsafe settings")
    for name, value in (
        ("ARMING_CHECK", 0),
        ("FS_THR_ENABLE", 0),
    ):
        try:
            vehicle.parameters[name] = value
        except Exception as error:
            print(f"SITL parameter {name} not applied: {error}")
    time.sleep(0.5)


def arm_in_stabilize(vehicle, rc: RCController | None = None):
    print("Switching to STABILIZE mode...")
    vehicle.mode = VehicleMode("STABILIZE")
    while vehicle.mode.name != "STABILIZE":
        time.sleep(0.2)

    # In STABILIZE, arming is often rejected if throttle is not low.
    if rc is not None:
        rc.reset_neutral(throttle=1000)
        time.sleep(0.5)

    print("Waiting until armable...")
    while not vehicle.is_armable:
        time.sleep(0.5)

    print("Arming motors...")
    arm_t0 = time.time()
    while not vehicle.armed and time.time() - arm_t0 < 8.0:
        if rc is not None:
            rc.send(roll=1500, pitch=1500, throttle=1000, yaw=1500, dt=0.2)
        vehicle.armed = True
        time.sleep(0.2)

    if not vehicle.armed:
        print("Normal arming blocked, trying force-arm...")
        vehicle._master.mav.command_long_send(
            vehicle._master.target_system,
            vehicle._master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            21196,
            0,
            0,
            0,
            0,
            0,
        )

    wait_t0 = time.time()
    while not vehicle.armed and time.time() - wait_t0 < 10.0:
        if rc is not None:
            rc.send(roll=1500, pitch=1500, throttle=1000, yaw=1500, dt=0.2)
        time.sleep(0.2)

    if not vehicle.armed:
        raise RuntimeError("Unable to arm vehicle")

    print("Armed")


def manual_takeoff(vehicle, rc: RCController, cfg: FlightConfig):
    print(f"Manual takeoff to ~{cfg.target_alt_m:.1f} m using RC override")
    cfg.alt_takeoff_pid.reset()
    cfg.alt_cruise_pid.reset()
    start_t = time.time()

    while True:
        alt = vehicle.location.global_relative_frame.alt or 0.0
        err = cfg.target_alt_m - alt

        if alt >= cfg.target_alt_m * 0.97:
            print(f"Takeoff target reached: alt={alt:.1f}m")
            break

        if time.time() - start_t > cfg.takeoff_timeout_s:
            raise TimeoutError("Takeoff timeout")

        throttle_corr = cfg.alt_takeoff_pid.update(err, cfg.loop_dt_s)
        if alt < 8.0:
            base = 1575
        elif alt < cfg.target_alt_m * 0.70:
            base = 1520
        elif alt < cfg.target_alt_m * 0.85:
            base = 1500
        else:
            base = 1475  # decelerate near target
        throttle = clamp(base + throttle_corr, 1475, 1680)
        throttle = apply_altitude_safety_cap(throttle, alt, cfg)

        rc.send(roll=1500, pitch=1500, throttle=throttle, yaw=1500, dt=cfg.loop_dt_s)

        print(f"Takeoff | alt={alt:6.1f}m err={err:6.1f} throttle={int(throttle)}")
        time.sleep(cfg.loop_dt_s)

    # Settle altitude: hover in place until within 2 m of target (max 60 s)
    # Use actual hover throttle (~1390) as neutral so corrections are immediate
    print("Post-takeoff altitude settle (hover until ~100 m)...")
    cfg.alt_cruise_pid.reset()
    settle_t0 = time.time()
    while time.time() - settle_t0 < 60.0:
        alt = vehicle.location.global_relative_frame.alt or 0.0
        err = cfg.target_alt_m - alt
        throttle = clamp(1390 + cfg.alt_cruise_pid.update(err, cfg.loop_dt_s), 1250, 1580)
        throttle = apply_altitude_safety_cap(throttle, alt, cfg)
        rc.send(roll=1500, pitch=1500, throttle=throttle, yaw=1500, dt=cfg.loop_dt_s)
        print(f"Settle | alt={alt:6.1f}m err={err:5.1f} thr={int(throttle)}")
        if abs(err) <= 2.0:
            print(f"Altitude settled at {alt:.1f}m, proceeding to cruise")
            break
        time.sleep(cfg.loop_dt_s)

def cruise_to_target(vehicle, rc: RCController, cfg: FlightConfig):
    print("Cruise phase in STABILIZE with RC override")

    cfg.alt_cruise_pid.reset()
    cfg.cruise_x_pid.reset()
    cfg.cruise_y_pid.reset()
    t0 = time.time()

    while True:
        if time.time() - t0 > cfg.cruise_timeout_s:
            raise TimeoutError("Cruise timeout")

        loc = vehicle.location.global_relative_frame
        lat = loc.lat
        lon = loc.lon
        alt = loc.alt or 0.0
        hdg = float(vehicle.heading or 0.0)

        dist = distance_m(lat, lon, cfg.target_lat, cfg.target_lon)
        if dist <= cfg.arrival_radius_m:
            print(f"Reached target radius: {dist:.1f} m")
            break

        desired_bearing = bearing_deg(lat, lon, cfg.target_lat, cfg.target_lon)
        n_err, e_err = relative_ne_m(lat, lon, cfg.target_lat, cfg.target_lon)

        yaw_rad = math.radians(hdg)
        forward_err = n_err * math.cos(yaw_rad) + e_err * math.sin(yaw_rad)
        right_err = -n_err * math.sin(yaw_rad) + e_err * math.cos(yaw_rad)

        alt_err = cfg.target_alt_m - alt
        throttle = clamp(
            1390 + cfg.alt_cruise_pid.update(alt_err, cfg.loop_dt_s),
            1280,
            1590,
        )
        throttle = apply_altitude_safety_cap(throttle, alt, cfg)

        dist_scale = 1.0
        if dist < cfg.slow_radius_m:
            dist_scale = clamp(dist / cfg.slow_radius_m, 0.35, 1.0)

        if dist < cfg.final_radius_m:
            dist_scale *= 0.8

        pitch_corr = cfg.cruise_x_pid.update(forward_err, cfg.loop_dt_s) * dist_scale
        roll_corr = cfg.cruise_y_pid.update(right_err, cfg.loop_dt_s) * dist_scale

        pitch = clamp(1500 - pitch_corr, 1340, 1660)
        roll = clamp(1500 + roll_corr, 1360, 1640)
        yaw = 1500

        rc.send(
            roll=roll,
            pitch=pitch,
            throttle=throttle,
            yaw=yaw,
            dt=cfg.loop_dt_s,
        )

        print(
            f"Cruise | dist={dist:6.1f}m alt={alt:6.1f}m hdg={hdg:6.1f} "
            f"bear={desired_bearing:6.1f} body=({forward_err:6.1f},{right_err:6.1f}) "
            f"rc(r,p,t,y)=({int(roll)},{int(pitch)},{int(throttle)},{int(yaw)})"
        )
        time.sleep(cfg.loop_dt_s)

    for _ in range(12):
        loc = vehicle.location.global_relative_frame
        alt = loc.alt or 0.0
        alt_err = cfg.target_alt_m - alt
        throttle = clamp(
            1390 + cfg.alt_cruise_pid.update(alt_err, cfg.loop_dt_s),
            1280,
            1560,
        )
        throttle = apply_altitude_safety_cap(throttle, alt, cfg)
        rc.send(roll=1500, pitch=1500, throttle=throttle, yaw=1500, dt=cfg.loop_dt_s)
        time.sleep(cfg.loop_dt_s)


def hover_align_at_target(vehicle, rc: RCController, cfg: FlightConfig):
    """Hover at cruise altitude and gently fly over target before descent."""
    print("Hover-align phase: gentle approach to target before descent")

    cfg.alt_cruise_pid.reset()
    cfg.align_x_pid.reset()
    cfg.align_y_pid.reset()

    t0 = time.time()
    align_timeout_s = 90.0
    align_target_dist_m = 4.0
    align_hold_s = 2.0
    align_in_radius_since = None

    while True:
        if time.time() - t0 > align_timeout_s:
            print("Hover-align timeout, proceeding with current position")
            break

        loc = vehicle.location.global_relative_frame
        lat = loc.lat
        lon = loc.lon
        alt = loc.alt or 0.0
        dist = distance_m(lat, lon, cfg.target_lat, cfg.target_lon)

        if dist <= align_target_dist_m:
            if align_in_radius_since is None:
                align_in_radius_since = time.time()
            if time.time() - align_in_radius_since >= align_hold_s:
                print(f"Hover-align complete: dist={dist:.2f}m")
                break
        else:
            align_in_radius_since = None

        hdg = float(vehicle.heading or 0.0)
        desired_bearing = bearing_deg(lat, lon, cfg.target_lat, cfg.target_lon)
        n_err, e_err = relative_ne_m(lat, lon, cfg.target_lat, cfg.target_lon)

        yaw_rad = math.radians(hdg)
        forward_err = n_err * math.cos(yaw_rad) + e_err * math.sin(yaw_rad)
        right_err = -n_err * math.sin(yaw_rad) + e_err * math.cos(yaw_rad)

        near_scale = clamp(dist / 8.0, 0.35, 1.0)
        pitch_corr = cfg.align_x_pid.update(forward_err, cfg.loop_dt_s) * near_scale
        roll_corr = cfg.align_y_pid.update(right_err, cfg.loop_dt_s) * near_scale

        pitch = clamp(1500 - pitch_corr, 1380, 1620)
        roll = clamp(1500 + roll_corr, 1380, 1620)
        yaw = 1500

        # Altitude hold
        alt_err = cfg.target_alt_m - alt
        throttle = clamp(
            1390 + cfg.alt_cruise_pid.update(alt_err, cfg.loop_dt_s),
            1280, 1580,
        )
        throttle = apply_altitude_safety_cap(throttle, alt, cfg)

        rc.send(roll=roll, pitch=pitch, throttle=throttle, yaw=yaw, dt=cfg.loop_dt_s)
        print(
            f"Align  | dist={dist:5.2f}m alt={alt:5.2f}m hdg={hdg:.1f} "
            f"bear={desired_bearing:.1f} body=({forward_err:5.2f},{right_err:5.2f}) "
            f"rc(r,p,t,y)=({int(roll)},{int(pitch)},{int(throttle)},{int(yaw)})"
        )
        time.sleep(cfg.loop_dt_s)

    # Brief hover stabilization
    for _ in range(10):
        loc = vehicle.location.global_relative_frame
        alt = loc.alt or 0.0
        alt_err = cfg.target_alt_m - alt
        throttle = clamp(
            1390 + cfg.alt_cruise_pid.update(alt_err, cfg.loop_dt_s),
            1280, 1560,
        )
        throttle = apply_altitude_safety_cap(throttle, alt, cfg)
        rc.send(roll=1500, pitch=1500, throttle=throttle, yaw=1500, dt=cfg.loop_dt_s)
        time.sleep(cfg.loop_dt_s)


def landing_at_target(vehicle, rc: RCController, cfg: FlightConfig):
    print("Landing phase: descent with stronger low-alt XY correction")

    cfg.land_x_pid.reset()
    cfg.land_y_pid.reset()

    heading_ref = float(vehicle.heading or 0.0)
    t0 = time.time()

    descent_target_alt = vehicle.location.global_relative_frame.alt or cfg.target_alt_m
    max_descent_rate = 2.4
    precision_touchdown_radius_m = min(cfg.touchdown_radius_m, 3.0)
    fallback_touchdown_radius_m = min(cfg.touchdown_radius_m, 3.6)

    while True:
        if time.time() - t0 > cfg.landing_timeout_s:
            raise TimeoutError("Landing timeout")

        loc = vehicle.location.global_relative_frame
        lat = loc.lat
        lon = loc.lon
        alt = loc.alt or 0.0
        dist = distance_m(lat, lon, cfg.target_lat, cfg.target_lon)

        # Prefer precise touchdown, but keep a safe fallback close to ground
        if (alt <= 2.0 and dist <= precision_touchdown_radius_m) or (alt <= 0.7 and dist <= fallback_touchdown_radius_m):
            break

        n_err, e_err = relative_ne_m(lat, lon, cfg.target_lat, cfg.target_lon)

        yaw_rad = math.radians(float(vehicle.heading or heading_ref))
        forward_err = n_err * math.cos(yaw_rad) + e_err * math.sin(yaw_rad)
        right_err = -n_err * math.sin(yaw_rad) + e_err * math.cos(yaw_rad)

        if alt > 30.0:
            xy_gain_scale = 0.55
        elif alt > 12.0:
            xy_gain_scale = 0.8
        elif alt > 2.5:
            xy_gain_scale = 1.75
        else:
            xy_gain_scale = 1.6

        yaw_rad = math.radians(float(vehicle.heading or heading_ref))
        pitch_corr = cfg.land_x_pid.update(forward_err, cfg.loop_dt_s) * xy_gain_scale
        roll_corr = cfg.land_y_pid.update(right_err, cfg.loop_dt_s) * xy_gain_scale

        pitch = clamp(1500 - pitch_corr, 1340, 1660)
        roll = clamp(1500 + roll_corr, 1340, 1660)
        yaw = 1500

        if alt > 25.0:
            descent_rate = max_descent_rate
        elif alt > 10.0:
            descent_rate = 1.7
        elif alt > 4.0:
            descent_rate = 0.6
        elif alt > 2.2:
            descent_rate = 0.30
        else:
            descent_rate = 0.0 if dist > precision_touchdown_radius_m else 0.35

        if dist > 5.0 and alt < 8.0:
            descent_rate *= 0.5

        descent_target_alt = max(0.0, descent_target_alt - descent_rate * cfg.loop_dt_s)
        alt_err = descent_target_alt - alt

        if alt > 20.0:
            throttle = 1330 + 15.0 * alt_err
            throttle = clamp(throttle, 1200, 1445)
        elif alt > 8.0:
            throttle = 1390 + 17.0 * alt_err
            throttle = clamp(throttle, 1260, 1470)
        elif alt > 2.2:
            throttle = 1400 + 28.0 * alt_err
            throttle = clamp(throttle, 1220, 1480)
        else:
            # Hold altitude until dist <= precision touchdown radius for final XY correction
            if dist > precision_touchdown_radius_m:
                throttle = 1500
            else:
                throttle = 1400 + 28.0 * alt_err
                throttle = clamp(throttle, 1220, 1480)

        rc.send(
            roll=roll,
            pitch=pitch,
            throttle=throttle,
            yaw=yaw,
            dt=cfg.loop_dt_s,
        )

        thr_p = 15.0 * alt_err if alt > 20.0 else (17.0 * alt_err if alt > 8.0 else 28.0 * alt_err)
        print(
            f"Land   | dist={dist:5.2f}m alt={alt:5.2f}m "
            f"pitch_PID=({cfg.land_x_pid.last_p:6.1f},{cfg.land_x_pid.last_i:5.1f},{cfg.land_x_pid.last_d:5.1f}) "
            f"roll_PID=({cfg.land_y_pid.last_p:6.1f},{cfg.land_y_pid.last_i:5.1f},{cfg.land_y_pid.last_d:5.1f}) "
            f"thr_P={thr_p:6.1f} alt_err={alt_err:5.2f} "
            f"rc(r,p,t)=({int(roll)},{int(pitch)},{int(throttle)})"
        )
        time.sleep(cfg.loop_dt_s)

    print("Touchdown detected. Cutting throttle and disarming...")
    for _ in range(12):
        rc.send(roll=1500, pitch=1500, throttle=1100, yaw=1500, dt=cfg.loop_dt_s)
        time.sleep(cfg.loop_dt_s)

    vehicle.armed = False
    for _ in range(25):
        if not vehicle.armed:
            break
        time.sleep(0.2)

    rc.clear()
    print("Landing complete")


def parse_args():
    parser = argparse.ArgumentParser(description="RC Override UAV script for SITL/ArduPilot")
    parser.add_argument("--connect", default="udp:127.0.0.1:14551", help="Vehicle connection string")
    parser.add_argument("--target-lat", type=float, default=50.443326)
    parser.add_argument("--target-lon", type=float, default=30.448078)
    parser.add_argument("--target-alt", type=float, default=300.0)
    parser.add_argument("--arrival-radius", type=float, default=25.0)
    parser.add_argument("--slow-radius", type=float, default=60.0)
    parser.add_argument("--wind-spd", type=float, default=5.0)
    parser.add_argument("--wind-dir", type=float, default=30.0)
    parser.add_argument("--wind-turb", type=float, default=3.0)
    parser.add_argument("--wind-turb-freq", type=float, default=0.25)
    return parser.parse_args()


def main():
    args = parse_args()

    cfg = FlightConfig(
        connect_str=args.connect,
        target_lat=args.target_lat,
        target_lon=args.target_lon,
        target_alt_m=args.target_alt,
        arrival_radius_m=args.arrival_radius,
        slow_radius_m=args.slow_radius,
        wind_spd=args.wind_spd,
        wind_dir=args.wind_dir,
        wind_turb=args.wind_turb,
        wind_turb_freq=args.wind_turb_freq,
    )

    vehicle = None
    rc = None

    try:
        print(f"Connecting to {cfg.connect_str} ...")
        vehicle = connect(cfg.connect_str, wait_ready=True, heartbeat_timeout=120)
        rc = RCController(vehicle, slew_per_sec=cfg.rc_slew_per_sec)

        wait_for_gps(vehicle)
        configure_sitl_vehicle(vehicle)
        configure_sim_wind(vehicle, cfg)

        start = vehicle.location.global_relative_frame
        print(f"Start point: lat={start.lat:.6f} lon={start.lon:.6f}")
        print(f"Target:      lat={cfg.target_lat:.6f} lon={cfg.target_lon:.6f} alt={cfg.target_alt_m:.1f}m")

        arm_in_stabilize(vehicle, rc)
        rc.reset_neutral(throttle=1000)

        manual_takeoff(vehicle, rc, cfg)
        cruise_to_target(vehicle, rc, cfg)
        hover_align_at_target(vehicle, rc, cfg)
        landing_at_target(vehicle, rc, cfg)

    finally:
        if rc is not None:
            try:
                rc.clear()
            except Exception:
                pass
        if vehicle is not None:
            try:
                vehicle.close()
            except Exception:
                pass


if __name__ == "__main__":
    main()