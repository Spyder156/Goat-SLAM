LaMAria native dual-camera global bundle adjustment
=================================================

This is an isolated Ceres solver linked against the existing COLMAP 4.2 build.
No third_party source is edited. It reads a COLMAP reconstruction with a fixed
two-camera metric rig and synchronized frames, then optimizes all supplied
registered rig poses and triangulated landmark positions in one global graph.

Build:
  python pipeline/vi_ba_lamaria/build.py

Run:
  bash pipeline/vi_ba_lamaria/run.sh \
    --input_path MODEL --output_path NEW_OUTPUT --settings_yaml SETTINGS \
    --imu_csv CALIBRATED_NATIVE_IMU --mode visual_rig \
    --max_iterations 30 --max_threads 4 --linear_solver iterative_schur

Modes (run each independently from the SAME source model):
  visual_rig  Visual observations, fixed supplied rig and intrinsics.
  vi_fixed    Above plus native IMU preintegration, velocity/bias states.
  vi_calib    Above plus all 15 native camera intrinsic parameters per camera.

Only cam0/<timestamp_ns>.* and cam1/<timestamp_ns>.* are accepted. Both images
at a timestamp must share one COLMAP frame, cam0 must be the rig reference,
and the sensor_from_rig transform must match inverse(Rig.T_c0_c1) in settings.
Each frame's pose is cam0_from_world. IMU.T_b_c1 is body_from_cam0. Quaternion
storage inside C++ is xyzw; COLMAP text uses wxyz. All 16 camera storage values
are retained: fx, fy, cx, cy, k0..k5, p0,p1,s0..s3. Native calibration has one
focal per camera so fx=fy, leaving 15 independent variables. Camera params and
feature coordinates must both use COLMAP's +0.5 pixel-centre convention.

The reprojection factor calls COLMAP's production RadTanThinPrismFisheyeModel;
there is no substituted pinhole/KB4/12-parameter approximation. All 15 lens
parameters have factory-centred Gaussian priors and total +/-3-sigma bounds.
Each accepted global solver iteration can move a camera parameter at most
0.15 sigma. Focal sigma is 1% of its supplied value (~2.42 px); principal-point
sigma is 1 px; radial sigmas are [.005,.0025,.0012,.0006,.0003,.00015];
tangential [.0005,.0005]; thin prism [.0005,.0002,.0005,.0002]. Camera inverse
domain is checked over the native 1.4-radian field after each step; an invalid
candidate restores the entire preceding state and is reported unusable.
These are shared full-sequence static parameters, NOT one lens per image.

Unprojectable trial observations follow COLMAP's production cost contract:
zero residual and no global evaluation failure. They are counted separately.
After optimization, unprojectable observations are deleted using COLMAP's
observation deletion API; landmarks with fewer than two observations disappear.
The exported graph and frame_support.csv describe this final retained graph.
More than max(100, 0.1% of input observations) unprojectable final observations
marks the result unusable. No observations are prefiltered out of an arm.

IMU input header:
  #timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z
The CSV must already contain native SDK-calibrated measurements in SI units.
The solver never recalibrates measurements a second time or changes clocks.
Every consecutive image interval is integrated over ALL native IMU samples,
with interpolation at the exact image timestamps. Intervals are not discarded
for being long. Missing endpoints or native sample gaps >10 ms fail explicitly.
Midpoint preintegration propagates 9D position/velocity/rotation covariance.
Its right attitude error is transported into the midpoint body frame, and
gyro-to-force noise uses the midpoint SO(3) right Jacobian. Rotating numerical
transition/covariance contracts verify these terms, including cross-covariances.
Each frame has its own gyro and accelerometer bias and velocity; adjacent biases
have random-walk factors using the supplied continuous-time noise densities.
Preintegrated bias derivatives use centred differences of the same integration.
Intervals are reintegrated when gyro bias changes >.002 rad/s or acceleration
bias >.02 m/s^2. Biases are held constant within each image interval.
Gravity direction is estimated from measured specific force and optimized on a
fixed-norm sphere (9.81 m/s^2); no assumption that the input world is Z-up.
The first pose fixes world gauge. The rig baseline stays physical and fixed.

An initial inertial state solve holds all image poses/landmarks/calibration
constant, adjusts velocities/biases/gravity, then releases geometry for global
optimization. result.json initial_cost is AFTER that state warmup; separate
initial_visual_cost / initial_imu_cost describe the original input states.
After the last update, all intervals are reintegrated at final biases for an
independent final audit. final_imu_cost/final_cost keep the accepted linearized
objective; final_imu_exact_cost/final_exact_cost report the exact reintegrated
objective. Maximum final gyro/accel bias linearization displacements and the
cost shift are reported. This audit is not counted as another accepted step.

Linear solver:
  --linear_solver sparse_schur: sparse direct SuiteSparse factorization.
  --linear_solver iterative_schur: Schur Jacobi, memory-safe implicit system.
  --linear_eta 0.001 --max_linear_iterations 1000 (defaults).
  --explicit_schur 1: assemble the reduced Schur matrix before iterative CG;
  same normal equations and Schur-Jacobi preconditioner, different memory/time
  tradeoff. This does not request sparse Cholesky factorization.
Loose eta=.1 left a synthetic VI problem visibly underconverged in 30 steps;
tighter settings recovered the reference to ~micrometre accuracy. Larger real
graphs can still require more iterations. Inspect converged, linear_capped_steps
and optimization.csv; a usable 30-step result is not a convergence guarantee.

The full 18,343-frame/26.56-million-observation graph exceeded a 26 GiB cgroup
limit before its first direct sparse-Schur step. That unscored profile is
preserved in the experiment suite. Full runs use implicit iterative Schur.
run.sh defaults to a 26 GiB limit with no additional swap and 4 CPUs; set
LAMARIA_BA_CPUS and --max_threads together to use a larger CPU allocation.
LAMARIA_BA_MEMORY changes the memory limit explicitly. Parent experiment
manifests record the chosen resource and linear-solver settings.
An optional solver_override.json beside an arm's model directory selects a
separately frozen binary, explicit-Schur setting, and resource limits. The
model/execution_manifest.json records actual binary/source/override hashes,
effective arguments and Docker command. This actual execution manifest takes
precedence over the suite's initial planned solver configuration. Replacing an
existing execution manifest requires archiving the preceding arm first.
The optional private Ceres2.2 build parallelizes only explicit Schur's symmetric
matrix-vector product using private partition outputs and ordered reduction.
Its exact9.26-million-block kernel benchmark measured3.31x speedup; this is a
kernel measurement, not an end-to-end solve claim. The isolated solver links
with an absolute private DT_RPATH; launcher overrides verify and record the
actual libceres hash in addition to the solver binary hash. System Ceres and
the original implicit visual executable remain unchanged.

Output:
  cameras/rigs/frames/images/points3D.txt: standard COLMAP text reconstruction.
  body_trajectory_ns.txt: timestamp_ns px py pz qx qy qz qw, world_from_body.
  imu_states.csv: timestamp_ns vx vy vz bgx bgy bgz bax bay baz.
  frame_support.csv: per-camera visual association counts, support category.
  calibration.json: input/final native 16-parameter arrays and 15 prior sigmas.
  optimization.csv: objective costs, reintegrations, calibration step size,
                    actual linear iterations and accepted/rejected step counts.
  result.json: coverage, optimizer/linear solver status and cost summaries.

No timestamps are synthesized, no poses interpolated, and no GT enters the
solver. Input timestamps with no visual observations remain unchanged in the
visual-only mode and are explicitly marked unchanged_input; in VI modes they
have IMU support and are marked imu_only. At least 20 cross-camera landmarks
are required to avoid presenting an unconstrained visual rig scale trial.
No descriptor extraction/rematching, loop proposal, or GT alignment happens in
this executable; the parent workflow prepares one common SIFT reconstruction.

Validation:
  python pipeline/vi_ba_lamaria/build.py --contracts
  docker run --rm --entrypoint '' -v "$PWD:/work" fisheye-slam \
    /work/build/colmap_lamaria_ba/contracts
  /home/raghav/miniconda3/envs/lamaria/bin/python \
    pipeline/vi_ba_lamaria/test_end_to_end.py

17 factor/coordinate contracts passed, including arbitrary world gravity,
nonidentity body-camera rotation/lever arm, derivative checks (max relative
error 1.3e-8), native image boundary interpolation, exact model domain,
fixed/full factor equivalence, and native negative-depth residual handling.
Independent 21-frame/42-image/20-interval synthetic global tests recover a
12% injected scale error in all three arms (direct solver). Preserve the test
outputs; the script refuses to overwrite completed per-arm results.

Optional observation metadata paging:
  --release_observation_metadata 1 (or the same boolean launcher override)
  writes an owned exact binary cache, then releases only image feature arrays
  and landmark track vectors after every residual and support count exists.
  Pose/XYZ nodes and Ceres parameter addresses stay alive. Metadata is streamed
  back before final geometry checks, pruning and text export. The cache hash,
  an independently recomputed hash of restored in-memory values, counters and
  reciprocal associations must agree or the run fails. This does not remove,
  reweight, deduplicate or replace any optimization measurement.
  observation_metadata.json records exact counts, hashes and released vector
  capacity; observation_metadata.cache is a local same-process binary artifact.

Four additional metadata contracts cover unmatched features, distinct feature
indices from one image in a landmark track, exact XY/ownership/counter/track
restoration, stable pose/XYZ addresses and updated geometry, and rejection of
a truncated cache. Release-on/off versions of all three global fixtures pass;
largest body-pose difference is 2.91e-11 and largest IMU-state difference2.75e-9.
Use build.py --metadata-contracts and pass an existing small fixture model to
the metadata_contracts executable. With private Ceres, Docker must mount the
workspace at its same absolute path so DT_RPATH resolves the pinned library.
