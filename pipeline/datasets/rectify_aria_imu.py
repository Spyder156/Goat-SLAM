#!/usr/bin/env python3
"""Convert raw Aria IMU CSV to factory-rectified, camera-clock measurements.

Input/output columns: timestamp_ns, wx, wy, wz, ax, ay, az (rad/s and m/s²).
Uses VRS factory calibration only; no trajectory or ground truth is required.

Aria defines actual(t) = rectified_raw(t + factory_offset + half_sample_period).
Gyro timestamps are shifted accordingly; accelerometer measurements are aligned
to that grid using their own calibration offset and linear interpolation.

References:
https://facebookresearch.github.io/projectaria_tools/docs/tech_insights/temporal_alignment_of_sensor_data
https://raw.githubusercontent.com/facebookresearch/projectaria_tools/main/core/calibration/ImuMagnetometerCalibration.cpp
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile

import numpy as np

TIMING_REFERENCE = "https://facebookresearch.github.io/projectaria_tools/docs/tech_insights/temporal_alignment_of_sensor_data"
MODEL_REFERENCE = "https://raw.githubusercontent.com/facebookresearch/projectaria_tools/main/core/calibration/ImuMagnetometerCalibration.cpp"
CSV_DTYPE = np.dtype([("timestamp_ns", "i8"), ("gyro", "f8", (3,)), ("accel", "f8", (3,))])


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def load_imu_csv(path):
    rows = np.loadtxt(path, delimiter=",", comments="#", dtype=CSV_DTYPE, ndmin=1)
    return rows["timestamp_ns"], rows["gyro"], rows["accel"]


def extract_affine(rectifier, probes):
    """Recover the SDK's affine correction and check it on sensor samples."""
    offset = np.asarray(rectifier(np.zeros(3)), dtype=float)
    matrix = np.column_stack([rectifier(axis) - offset for axis in np.eye(3)])
    expected = np.stack([rectifier(sample) for sample in probes])
    error = float(np.max(np.abs(np.asarray(probes) @ matrix.T + offset - expected)))
    if not np.isfinite(matrix).all() or not np.isfinite(offset).all() or error > 1e-10:
        raise ValueError(f"SDK rectification is not the expected affine model: error={error}")
    return matrix, offset, error


def rectify_and_align(timestamps_ns, gyro, accel, gyro_affine, accel_affine,
                      gyro_offset_s, accel_offset_s, rate_hz):
    """Return integer timestamps, corrected channels and retained source indices.

    No extrapolation is allowed: output is the corrected gyro grid restricted
    to the temporal overlap with the independently corrected accel grid.
    """
    timestamps_ns = np.asarray(timestamps_ns)
    gyro = np.asarray(gyro, dtype=float)
    accel = np.asarray(accel, dtype=float)
    if timestamps_ns.dtype.kind not in "iu":
        raise ValueError("Timestamps must be integer nanoseconds")
    if len(timestamps_ns) < 2 or np.any(np.diff(timestamps_ns) <= 0):
        raise ValueError("At least two strictly increasing timestamps are required")
    if gyro.shape != (len(timestamps_ns), 3) or accel.shape != gyro.shape:
        raise ValueError("Each timestamp must have three gyro and three accel values")
    if not np.isfinite(gyro).all() or not np.isfinite(accel).all():
        raise ValueError("Non-finite sensor values")
    if not np.isfinite([rate_hz, gyro_offset_s, accel_offset_s]).all() or rate_hz <= 0:
        raise ValueError("Invalid nominal sample rate or factory time offset")
    gyro_delay = gyro_offset_s + 0.5 / rate_hz
    accel_delay = accel_offset_s + 0.5 / rate_hz
    # Subtract an integer epoch before conversion to seconds to retain precision.
    relative_t = (timestamps_ns - timestamps_ns[0]) * 1e-9
    gyro_t = relative_t - gyro_delay
    accel_t = relative_t - accel_delay
    keep = np.flatnonzero((gyro_t >= accel_t[0]) & (gyro_t <= accel_t[-1]))
    if len(keep) < 2:
        raise ValueError("Fewer than two samples in the common calibrated time interval")
    matrix_g, offset_g = gyro_affine
    matrix_a, offset_a = accel_affine
    corrected_gyro = gyro @ np.asarray(matrix_g).T + offset_g
    corrected_accel = accel @ np.asarray(matrix_a).T + offset_a
    aligned_accel = np.column_stack([
        np.interp(gyro_t[keep], accel_t, corrected_accel[:,axis]) for axis in range(3)
    ])
    output_t = timestamps_ns[keep] - round(gyro_delay * 1e9)
    if not np.isfinite(corrected_gyro[keep]).all() or not np.isfinite(aligned_accel).all():
        raise ValueError("Non-finite calibrated output")
    return output_t, corrected_gyro[keep], aligned_accel, keep


def write_csv(path, timestamps_ns, gyro, accel, force=False):
    """Write atomically, retaining exact integer timestamps even beyond2**53."""
    path = Path(path)
    if path.exists() and not force:
        raise FileExistsError(f"Output exists; pass --force to replace it: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, prefix=path.name+".", delete=False) as handle:
            temp_path = Path(handle.name)
            handle.write("#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z\n")
            for start in range(0, len(timestamps_ns), 65536):
                end = min(start+65536, len(timestamps_ns))
                handle.writelines(
                    str(int(t)) + "," + ",".join(format(float(x), ".12g") for x in (*w, *a)) + "\n"
                    for t, w, a in zip(timestamps_ns[start:end], gyro[start:end], accel[start:end])
                )
        if force:
            os.replace(temp_path, path)
        else:
            # Hard-link creation is atomic and refuses a concurrently created output.
            os.link(temp_path, path)
            temp_path.unlink()
        temp_path = None
    finally:
        if temp_path is not None:
            temp_path.unlink(missing_ok=True)


def run(args):
    source = args.input_csv.resolve()
    destination = args.output_csv.resolve()
    manifest_path = args.output_csv.with_suffix(args.output_csv.suffix + ".manifest.json")
    if source == destination:
        raise ValueError("Input and output must be different paths, including with --force")
    if not args.force and (args.output_csv.exists() or manifest_path.exists()):
        raise FileExistsError("Output CSV or manifest exists; pass --force to replace it")
    if args.verify_samples < 0:
        raise ValueError("--verify-samples must be nonnegative")
    # Lazy import keeps numerical helpers and unit tests independent of the SDK.
    from projectaria_tools.core import data_provider
    from projectaria_tools.core.sensor_data import TimeDomain, TimeQueryOptions

    provider = data_provider.create_vrs_data_provider(str(args.vrs))
    sid = provider.get_stream_id_from_label(args.imu_label)
    calibration = provider.get_device_calibration().get_imu_calib(args.imu_label)
    if calibration is None:
        raise ValueError(f"No factory calibration for {args.imu_label}")
    configuration = provider.get_imu_configuration(sid)
    rate = float(configuration.nominal_rate_hz)
    timestamps, gyro, accel = load_imu_csv(args.input_csv)
    probes = np.linspace(0, len(timestamps)-1, min(max(args.verify_samples, 1), len(timestamps)), dtype=int)
    timestamp_error = 0
    if args.verify_samples:
        for index in probes:
            native = provider.get_imu_data_by_time_ns(
                sid, int(timestamps[index]), TimeDomain.DEVICE_TIME, TimeQueryOptions.CLOSEST
            )
            delta = abs(int(native.capture_timestamp_ns)-int(timestamps[index]))
            # VRS→ROS conversion can round timestamps by1ns.
            if delta > 1:
                raise ValueError(f"CSV sample {index} does not match native VRS timestamp (difference {delta}ns)")
            if not np.allclose(native.gyro_radsec, gyro[index], rtol=1e-7, atol=1e-9) or not np.allclose(native.accel_msec2, accel[index], rtol=1e-7, atol=1e-8):
                raise ValueError(f"CSV sample {index} is not raw {args.imu_label} data; refusing possible double calibration")
            timestamp_error = max(timestamp_error, delta)
    g_matrix, g_offset, g_error = extract_affine(calibration.raw_to_rectified_gyro, gyro[probes])
    a_matrix, a_offset, a_error = extract_affine(calibration.raw_to_rectified_accel, accel[probes])
    dg = float(calibration.get_time_offset_sec_device_gyro())
    da = float(calibration.get_time_offset_sec_device_accel())
    t, w, a, keep = rectify_and_align(timestamps, gyro, accel, (g_matrix,g_offset),
                                     (a_matrix,a_offset), dg, da, rate)
    coverage = None
    if args.camera_timestamps:
        camera_t = np.loadtxt(args.camera_timestamps, dtype=np.int64, ndmin=1)
        if camera_t.ndim != 1 or len(camera_t) == 0:
            raise ValueError("--camera-timestamps must contain one integer timestamp per line")
        coverage = bool(t[0] < camera_t.min() and t[-1] > camera_t.max())
        if not coverage:
            raise ValueError("Calibrated IMU samples do not bracket every requested camera timestamp")
    write_csv(args.output_csv, t, w, a, force=args.force)
    intervals = np.diff(t)
    manifest = {
        "vrs": str(args.vrs.resolve()), "input_csv": str(source), "output_csv": str(destination),
        "input_sha256": sha256(args.input_csv), "output_sha256": sha256(args.output_csv),
        "imu_label": args.imu_label, "stream_id": str(sid), "nominal_rate_hz": rate,
        "factory_gyro_offset_s": dg, "factory_accel_offset_s": da,
        "half_sample_period_s": 0.5/rate, "gyro_total_delay_s": dg+0.5/rate, "accel_total_delay_s": da+0.5/rate,
        "timing_equation": "actual(t_device)=interpolated_rectified_raw(t_device+factory_offset+0.5/rate)",
        "output_grid": "gyro capture timestamp minus gyro_total_delay, rounded to integer nanoseconds",
        "accel_resampling": "linear interpolation on the independently corrected accel grid; no extrapolation",
        "intrinsics_equation": "rectified=A*raw+offset=rectificationMatrix.inverse()*(raw-factory_bias)",
        "gyro_affine": {"A": g_matrix.tolist(), "offset": g_offset.tolist()},
        "accel_affine": {"A": a_matrix.tolist(), "offset": a_offset.tolist()},
        "T_device_imu": calibration.get_transform_device_imu().to_matrix().tolist(),
        "native_raw_samples_verified": len(probes) if args.verify_samples else 0,
        "native_timestamp_max_abs_error_ns": timestamp_error if args.verify_samples else None,
        "native_sdk_max_abs_rectification_error": {"gyro": g_error, "accel": a_error},
        "input_rows": len(timestamps), "output_rows": len(t), "dropped_boundary_rows": len(timestamps)-len(t),
        "first_timestamp_ns": int(t[0]), "last_timestamp_ns": int(t[-1]),
        "interval_ns": {"min": int(intervals.min()), "median": float(np.median(intervals)), "max": int(intervals.max())},
        "camera_timestamps_covered": coverage,
        "sources": [TIMING_REFERENCE, MODEL_REFERENCE], "ground_truth_used": False,
    }
    with manifest_path.open("w" if args.force else "x") as handle:
        json.dump(manifest, handle, indent=2)
        handle.write("\n")
    print(f"Wrote {len(t)} calibrated samples to {args.output_csv}; manifest: {manifest_path}")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vrs", required=True, type=Path)
    parser.add_argument("--input-csv", required=True, type=Path)
    parser.add_argument("--output-csv", required=True, type=Path)
    parser.add_argument("--imu-label", default="imu-right")
    parser.add_argument("--camera-timestamps", type=Path, help="Optional one-column timestamp_ns list; require complete IMU coverage")
    parser.add_argument("--verify-samples", type=int, default=128, help="Check raw CSV against native VRS (default128;0 disables)")
    parser.add_argument("--force", action="store_true", help="Replace an existing output CSV and its manifest")
    args = parser.parse_args()
    try:
        run(args)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()
