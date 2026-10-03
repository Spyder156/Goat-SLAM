#!/usr/bin/env python3
"""Plot a completed native-fisheye VI calibration solve without changing its output."""

import argparse
import csv
import hashlib
import html
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pycolmap


MODEL = "RAD_TAN_THIN_PRISM_FISHEYE"
NAMES = ["f", "cx", "cy"] + [f"k{i}" for i in range(6)] + ["p0", "p1"] + [f"s{i}" for i in range(4)]
INDEPENDENT = [0] + list(range(2, 16))


def read_json(path):
    return json.loads(path.read_text())


def provenance(path):
    return {"path": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def parameters(values):
    values = np.asarray(values, dtype=float)
    if values.shape != (16,) or not np.isfinite(values).all():
        raise ValueError("Expected 16 finite native Fisheye624 parameters")
    if not np.isclose(values[0], values[1], rtol=0, atol=1e-10):
        raise ValueError("This report expects one shared focal length per camera")
    return values


def read_camera_metadata(path):
    cameras = {}
    for line in path.read_text().splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        cid, model, width, height, *values = line.split()
        if model != MODEL:
            raise ValueError(f"Unsupported exported camera: {model}")
        cameras[int(cid)] = (int(width), int(height), parameters(values))
    return cameras


def make_camera(colmap_parameters, width, height):
    # Batch camera arrays ALREADY use COLMAP pixel centres. Do not shift twice.
    return pycolmap.Camera(model=MODEL, width=width, height=height, params=colmap_parameters)


def field_stats(values):
    if len(values) == 0:
        return {"count": 0, "median_px": None, "p95_px": None, "maximum_px": None}
    return {"count": len(values), "median_px": float(np.median(values)),
            "p95_px": float(np.percentile(values, 95)), "maximum_px": float(np.max(values))}


def projection_fields(cameras, output):
    comparisons = [("factory", "batch", "Batch final vs factory")]
    if "online" in cameras[0]:
        comparisons += [("factory", "online", "Online v2 final vs factory"),
                        ("online", "batch", "Batch final vs online v2")]
    fig, axes = plt.subplots(2, len(comparisons), figsize=(6 * len(comparisons), 9),
                             squeeze=False, constrained_layout=True)
    panels, measurements = [], []
    for camera in cameras:
        cam, width, height = camera["cam_index"], camera["width"], camera["height"]
        factory = make_camera(camera["factory"], width, height)
        u, v = np.meshgrid(np.arange(4, width, 8), np.arange(4, height, 8))
        pixels = np.column_stack((u.ravel(), v.ravel()))
        tangent = factory.cam_from_img(pixels + .5)
        rays = np.column_stack((tangent, np.ones(len(tangent))))
        theta = np.arctan(np.linalg.norm(tangent, axis=1))
        physical_centre = camera["factory"][2:4] - .5
        valid = ((np.linalg.norm(pixels - physical_centre, axis=1) <= 330)
                 & (theta <= 1.4) & np.isfinite(rays).all(axis=1))
        rays, pixels, theta = rays[valid], pixels[valid], theta[valid]
        if not len(rays):
            raise ValueError("No admissible factory rays")
        projected = {key: make_camera(camera[key], width, height).img_from_cam(rays) - .5
                     for key in ("factory", "batch", "online") if key in camera}
        roundtrip = float(np.max(np.linalg.norm(projected["factory"] - pixels, axis=1)))
        if roundtrip > 1e-6:
            raise ValueError(f"Factory production-model round trip failed: {roundtrip}px")
        for col, (first, second, label) in enumerate(comparisons):
            delta = projected[second] - projected[first]
            magnitude = np.linalg.norm(delta, axis=1)
            if not np.isfinite(magnitude).all():
                raise ValueError("Non-finite projection displacement")
            measurements.append({"cam_index": cam, "comparison": label,
                                 "all": field_stats(magnitude),
                                 "central": field_stats(magnitude[theta <= .5]),
                                 "peripheral": field_stats(magnitude[theta >= 1.1]),
                                 "max_sample_angle_rad": float(theta.max()),
                                 "excluded_grid_samples": int(len(valid) - np.count_nonzero(valid)),
                                 "factory_roundtrip_max_px": roundtrip})
            panels.append((axes[cam, col], camera, pixels, delta, magnitude, label))
    vmax = max(float(panel[4].max()) for panel in panels)
    for ax, camera, pixels, delta, magnitude, label in panels:
        scatter = ax.scatter(pixels[:, 0], pixels[:, 1], c=magnitude, marker="s", s=16,
                             cmap="magma", vmin=0, vmax=max(vmax, 1e-12))
        take = np.arange(len(pixels))[::24]
        ax.quiver(pixels[take, 0], pixels[take, 1], delta[take, 0], delta[take, 1],
                  color="cyan", angles="xy", scale_units="xy", scale=.15, width=.002)
        ax.set(xlim=(0, camera["width"]), ylim=(camera["height"], 0),
               xlabel="Native u (px)", ylabel="Native v (px)",
               title=f"cam{camera['cam_index']}: {label}")
        ax.set_aspect("equal")
    fig.colorbar(scatter, ax=axes, label="Projection change (px); arrows enlarged 6.67×")
    fig.suptitle("Identical factory rays, including the periphery: projection change ≠ GT error")
    fig.savefig(output / "projection_changes.png", dpi=150)
    plt.close(fig)
    return measurements


def plot_parameters(cameras, bound, output):
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), constrained_layout=True)
    x = np.arange(15)
    for camera, ax in zip(cameras, axes):
        ax.axhspan(-bound, bound, color="#d9ecdf", alpha=.7, label="Batch lifetime bounds")
        ax.axhline(bound, color="#578464", linestyle="--")
        ax.axhline(-bound, color="#578464", linestyle="--")
        ax.axhline(0, color="black", linewidth=.6)
        ax.bar(x, camera["delta_sigma"], color="#2473ac", width=.65, label="Batch final")
        if "online_delta_sigma" in camera:
            ax.plot(x, camera["online_delta_sigma"], "D", color="#cc732a",
                    label="Online v2 reference (different priors)")
        ax.set(xticks=x, xticklabels=NAMES, ylabel="Change / batch factory-prior σ",
               title=f"cam{camera['cam_index']}: 15 independent parameters (fx = fy = f)")
        ax.grid(axis="y", alpha=.2)
        ax.legend(loc="upper left", bbox_to_anchor=(1.005, 1), fontsize=9)
    fig.suptitle("Final camera calibration relative to the frozen factory values")
    fig.savefig(output / "parameter_changes.png", dpi=150)
    plt.close(fig)


def plot_history(rows, result, cap, output):
    iteration = np.array([row["iteration"] + 1 for row in rows])
    accepted = np.array([bool(row["accepted"]) for row in rows])
    step = np.array([row["max_intrinsic_step_sigma"] for row in rows])
    fig, axes = plt.subplots(2, 1, figsize=(13, 7), sharex=True, constrained_layout=True)
    axes[0].axhline(cap, color="#ac3939", linestyle="--", label=f"Per-attempt cap: {cap:g}σ")
    axes[0].plot(iteration, step, color="#777", linewidth=1)
    for mask, label, color, marker in [(accepted, "Accepted update", "#2473ac", "o"),
                                       (~accepted, "No accepted update", "#b15a2a", "x")]:
        if mask.any():
            axes[0].scatter(iteration[mask], step[mask], c=color, marker=marker, label=label)
    axes[0].set(ylabel="Maximum parameter step (σ)", ylim=(-.01 * cap, max(cap, step.max()) * 1.15),
                title="Largest absolute camera-parameter change in each outer attempt")
    axes[0].legend()
    axes[1].bar(iteration, [row["linear_iterations"] for row in rows], color="#63869c")
    axes[1].set(xlabel="Outer attempt (1-based)", ylabel="Linear solver iterations")
    for ax in axes:
        ax.grid(axis="y", alpha=.2)
    fig.savefig(output / "step_limits.png", dpi=150)
    plt.close(fig)

    total = np.array([row["cost"] for row in rows])
    visual = np.array([row["visual_cost"] for row in rows])
    imu = np.array([row["imu_cost"] for row in rows])
    other = total - visual - imu
    if np.any(other < -np.maximum(1., np.abs(total)) * 1e-8):
        raise ValueError("Objective components exceed the total beyond round-off tolerance")
    other = np.maximum(other, 0)
    fig, ax = plt.subplots(figsize=(13, 6), constrained_layout=True)
    for values, label, color in [(total, "Total", "#252525"), (visual, "Visual (robust loss)", "#2473ac"),
                                  (imu, "IMU (current bias linearization)", "#b86f23"),
                                  (other, "Other priors / bias random walk (by subtraction)", "#679453")]:
        ax.plot(iteration, np.where(values > 0, values, np.nan), "o-", markersize=4,
                label=label, color=color)
    ax.scatter([0], [result["initial_cost"]], marker="s", color="black", s=60,
               label="Total before first attempt (after inertial warmup)")
    if result.get("final_exact_cost", 0) > 0:
        ax.scatter([iteration[-1]], [result["final_exact_cost"]], marker="*", s=160,
                   facecolors="none", edgecolors="#d53868", linewidths=1.5,
                   label="Final exact IMU reintegration audit (no optimization)")
    ax.set(yscale="log", xlabel="Outer attempt (1-based; 0 = initial total)", ylabel="Objective cost",
           title="Optimization history; objective reduction is not a ground-truth score")
    ax.grid(alpha=.2)
    ax.legend(fontsize=9)
    fig.savefig(output / "objective_components.png", dpi=150)
    plt.close(fig)


def table(headers, rows):
    return "<table><thead><tr>" + "".join(f"<th>{html.escape(str(x))}</th>" for x in headers) + \
        "</tr></thead><tbody>" + "".join("<tr>" + "".join(f"<td>{html.escape(str(x))}</td>" for x in row)
                                         + "</tr>" for row in rows) + "</tbody></table>"


def write_html(report, cameras, title, output):
    result = report["result"]
    summary = report["summary"]
    sections = []
    sections.append("<h2>Completed solve</h2>" + table(["Metric", "Value"], [
        ("Frames / observations", f"{result['frames']:,} / {result['observations']:,}"),
        ("Usable / converged", f"{result['usable']} / {result.get('converged', 'unreported')}"),
        ("Accepted / total outer attempts", f"{summary['accepted_attempts']} / {summary['total_attempts']}"),
        ("Initial → final objective", f"{result['initial_cost']:.8g} → {result['final_cost']:.8g}"),
        ("Maximum attempted intrinsic step", f"{summary['max_step_sigma']:.9g}σ (cap {report['max_step_sigma']:g}σ)"),
        ("Largest final intrinsic change", f"{summary['max_final_parameter_sigma']:.9g}σ (lifetime ±{report['max_total_sigma']:g}σ)"),
        ("Bounds respected", summary['bounds_respected']),
        ("Linear iterations at cap", result.get("linear_capped_steps", "unreported")),
    ]))
    sections.append("<h2>Camera parameter changes</h2><p>Both cameras retain the native Fisheye624 model. "
                    "Focal length is shared between x and y within each camera, leaving 15 independent parameters. "
                    "The shaded band is the batch solver’s lifetime bound. Online v2 markers are a reference, "
                    "normalized using the batch sigmas; that run used different bounds.</p>"
                    '<img src="parameter_changes.png" alt="Final calibration parameter changes">')
    for cam in cameras:
        values = [(name, f"{cam['factory'][idx]:.10g}", f"{cam['batch'][idx]:.10g}",
                   f"{cam['batch'][idx] - cam['factory'][idx]:+.6g}", f"{cam['sigma'][k]:.6g}",
                   f"{cam['delta_sigma'][k]:+.6f}") for k, (name, idx) in enumerate(zip(NAMES, INDEPENDENT))]
        sections.append(f"<h3>cam{cam['cam_index']}</h3>" + table(
            ["Parameter", "Factory (COLMAP centres)", "Final", "Change", "Factory-prior σ", "Change / σ"], values))
    sections.append("<h2>Step limits and objective</h2><p>A successful bounded solve should keep every "
                    "attempt below the step cap and the final parameters inside the lifetime bounds. "
                    "The CSV stores only the maximum per-parameter step in each attempt, not a full parameter "
                    "history. A nonaccepted attempt may be convergence without movement. Costs can change when "
                    "IMU factors are reintegrated; lower training cost alone does not establish better calibration "
                    "or a better leaderboard score.</p>"
                    '<img src="step_limits.png" alt="Calibration step bounds and linear solver work">'
                    '<img src="objective_components.png" alt="Visual and inertial objective histories">')
    audit_keys = ["final_imu_linearized_cost", "final_imu_exact_cost", "final_exact_cost",
                  "final_imu_cost_relative_shift", "final_max_gyro_bias_linearization_displacement",
                  "final_max_accel_bias_linearization_displacement", "final_exact_reintegrations"]
    sections.append("<h3>Final exact IMU audit</h3><p>These values evaluate the accepted state after "
                    "reintegrating IMU measurements at the final biases. This audit performs no additional "
                    "optimization. Gyro bias is rad/s; accelerometer bias is m/s².</p>" + table(
                        ["Metric", "Value"], [(key, result[key]) for key in audit_keys if key in result]))
    sections.append("<h2>Projection changes, including the periphery</h2><p><strong>Projection change ≠ "
                    "ground-truth error.</strong> Each panel projects exactly the same factory rays through "
                    "different accepted camera models using production pycolmap. The factory mask is frozen; "
                    "it does not move with the optimized principal point. Samples cover the physical 330 px "
                    "disk and θ ≤ 1.4 rad. Central samples have θ ≤ 0.5 rad; peripheral samples have θ ≥ 1.1 rad. "
                    "A good result has finite, smooth fields; large or small displacement by itself cannot "
                    "prove accuracy. The COLMAP +0.5 pixel-centre shift is applied once to optional native "
                    "online parameters, never to the already shifted batch arrays.</p>"
                    '<img src="projection_changes.png" alt="Image-wide native fisheye projection displacement">')
    sections.append(table(["Camera", "Comparison", "Centre median px", "Periphery median px", "Overall p95 px", "Maximum px"],
                          [(f"cam{r['cam_index']}", r["comparison"], f"{r['central']['median_px']:.6f}",
                            f"{r['peripheral']['median_px']:.6f}", f"{r['all']['p95_px']:.6f}",
                            f"{r['all']['maximum_px']:.6f}") for r in report["projection_fields"]]))
    sections.append("<h2>Source records</h2><p>The report reads frozen completed results and does not change "
                    "the estimator, trajectories, or camera files. Exact parameters, measurements and source "
                    "hashes are in <a href='metrics.json'>metrics.json</a>.</p>" + table(
                        ["Input", "Path", "SHA256"], [(key, value['path'], value['sha256'])
                                                      for key, value in report['inputs'].items()]))
    document = """<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>""" + html.escape(title) + """</title><style>
body{font:16px/1.5 system-ui,sans-serif;color:#202a34;background:#f6f8fa;max-width:1450px;margin:35px auto;padding:0 24px}
h1,h2,h3{line-height:1.2}h2{margin-top:40px}img{display:block;width:100%;height:auto;background:white;margin:20px 0;border:1px solid #d8dfe5}
table{border-collapse:collapse;display:block;overflow:auto;background:white;margin:16px 0;font-size:14px}
td,th{border:1px solid #d8dfe5;padding:8px 12px;text-align:left;overflow-wrap:anywhere}th{background:#e8eef3}
p{max-width:1050px}code{overflow-wrap:anywhere}</style><h1>""" + html.escape(title) + "</h1>" + "".join(sections) + "</html>\n"
    (output / "report.html").write_text(document)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True, help="Completed vi_calib model directory")
    parser.add_argument("--online-metrics", type=Path)
    parser.add_argument("--completion-marker", type=Path, help="Optional solve.command_result.json; requires returncode=0")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--title", default="LaMAria full batch VI calibration diagnostics")
    args = parser.parse_args()
    paths = {key: args.model / name for key, name in [
        ("calibration", "calibration.json"), ("optimization", "optimization.csv"),
        ("result", "result.json"), ("cameras", "cameras.txt")]}
    if args.completion_marker:
        if read_json(args.completion_marker).get("returncode") != 0:
            raise ValueError("Solve completion marker is not successful")
        paths["completion_marker"] = args.completion_marker
    calibration, result = read_json(paths["calibration"]), read_json(paths["result"])
    if result.get("mode") != "vi_calib" or result.get("usable") is not True:
        raise ValueError("Report requires a completed usable vi_calib result")
    metadata = read_camera_metadata(paths["cameras"])
    with paths["optimization"].open(newline="") as stream:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(stream)]
    if not rows or not all(np.isfinite(list(row.values())).all() for row in rows):
        raise ValueError("Missing or non-finite optimization history")
    if [int(row["iteration"]) for row in rows] != list(range(len(rows))):
        raise ValueError("Optimization history is not a contiguous zero-based sequence")
    if sum(bool(row["accepted"]) for row in rows) != result["accepted_iterations"]:
        raise ValueError("CSV and final result disagree on accepted iterations")
    if not np.isclose(rows[-1]["cost"], result["final_cost"], rtol=1e-10, atol=1e-8):
        raise ValueError("CSV and final result disagree on final cost")
    online = read_json(args.online_metrics) if args.online_metrics else None
    if online:
        if not online.get("commits"):
            raise ValueError("Online metrics have no accepted calibration commit")
        paths["online_metrics"] = args.online_metrics
    cameras = []
    for row in sorted(calibration["cameras"], key=lambda row: row["cam_index"]):
        cam = row["cam_index"]
        factory, batch = parameters(row["initial"]), parameters(row["final"])
        sigma = np.asarray(row["sigma_15"], dtype=float)
        if sigma.shape != (15,) or not np.isfinite(sigma).all() or np.any(sigma <= 0):
            raise ValueError("Expected 15 positive finite prior sigmas")
        width, height, exported = metadata[row["camera_id"]]
        np.testing.assert_allclose(batch, exported, atol=1e-12, rtol=1e-12)
        camera = {"cam_index": cam, "camera_id": row["camera_id"], "width": width, "height": height,
                  "factory": factory, "batch": batch, "sigma": sigma,
                  "delta_sigma": (batch[INDEPENDENT] - factory[INDEPENDENT]) / sigma}
        if online:
            online_factory = parameters(online["factory_native_parameters"][cam]).copy()
            online_factory[2:4] += .5
            np.testing.assert_allclose(factory, online_factory, atol=1e-10, rtol=0,
                                       err_msg="Batch and online factory calibration differ")
            final = parameters(online["commits"][-1]["parameters"][str(cam)]).copy()
            final[2:4] += .5
            camera["online"] = final
            camera["online_delta_sigma"] = (final[INDEPENDENT] - factory[INDEPENDENT]) / sigma
        cameras.append(camera)
    if [camera["cam_index"] for camera in cameras] != [0, 1]:
        raise ValueError("Expected exactly cam0 and cam1")
    cap, bound = float(calibration["max_step_sigma"]), float(calibration["max_total_sigma"])
    args.output.mkdir(parents=True, exist_ok=True)
    plot_parameters(cameras, bound, args.output)
    plot_history(rows, result, cap, args.output)
    fields = projection_fields(cameras, args.output)
    max_step = max(row["max_intrinsic_step_sigma"] for row in rows)
    max_final = max(float(np.max(np.abs(camera["delta_sigma"]))) for camera in cameras)
    report = {"title": args.title, "inputs": {key: provenance(path) for key, path in paths.items()},
              "result": result, "parameter_names": NAMES, "independent_parameter_indices": INDEPENDENT,
              "max_step_sigma": cap, "max_total_sigma": bound, "optimization": rows,
              "summary": {"total_attempts": len(rows), "accepted_attempts": result["accepted_iterations"],
                          "max_step_sigma": max_step, "max_final_parameter_sigma": max_final,
                          "bounds_respected": bool(max_step <= cap + 1e-9 and max_final <= bound + 1e-9)},
              "cameras": [{key: value.tolist() if isinstance(value, np.ndarray) else value
                           for key, value in camera.items()} for camera in cameras],
              "projection_model": MODEL, "pycolmap_version": pycolmap.__version__,
              "projection_domain": {"native_pixel_grid_step": 8, "physical_radius_px": 330,
                                    "max_theta_rad": 1.4, "central_theta_max_rad": .5,
                                    "peripheral_theta_min_rad": 1.1},
              "pixel_convention": "Batch parameters already use COLMAP centres. Online native cx/cy receive +0.5 once. Plots use native pixels.",
              "interpretation": "Projection change is not GT error. Training objective is not an evaluation score.",
              "projection_fields": fields,
              "online_final_commit_s": online["commits"][-1]["t"] if online else None}
    (args.output / "metrics.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    write_html(report, cameras, args.title, args.output)
    print(json.dumps({"output": str(args.output.resolve()), **report["summary"]}, indent=2))
    for camera in cameras:
        k = int(np.argmax(np.abs(camera["delta_sigma"])))
        print(f"cam{camera['cam_index']}: largest final change {NAMES[k]} {camera['delta_sigma'][k]:+.6f}σ; "
              f"focal change {camera['batch'][0] - camera['factory'][0]:+.6f}px")
    for field in fields:
        print(f"cam{field['cam_index']} {field['comparison']}: central median {field['central']['median_px']:.6f}px; "
              f"peripheral median {field['peripheral']['median_px']:.6f}px; maximum {field['all']['maximum_px']:.6f}px")


if __name__ == "__main__":
    main()
