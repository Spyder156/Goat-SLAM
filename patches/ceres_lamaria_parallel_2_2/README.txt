Isolated Ceres 2.2 parallel explicit-Schur matvec
================================================

Scope
  Only SymmetricRightMultiplyAndAccumulate is changed in the numerical library.
  All public headers, public object layouts, loss functions, normal equations,
  trust-region settings and linear tolerances remain unchanged. The build
  disables Eigen METIS and CHOLMOD Partition to match installed Ubuntu Ceres.
  No third_party source, installed library or active binary was modified.
  This build did not edit launchers; later selection of verified future VI
  overrides is recorded by the parent solver workflow separately.

Source
  Official ceres-solver 2.2.0 archive:
  https://codeload.github.com/ceres-solver/ceres-solver/tar.gz/refs/tags/2.2.0
  SHA256 12efacfadbfdc1bbfa203c236e96f4d3c210bed96994288b3ff0c8e7c6f350d4
  Staging: build/ceres_lamaria_parallel/ceres-solver-2.2.0
  Private prefix: build/ceres_lamaria_parallel/install
  apply.py applies this patch and adds the contract/benchmark target to a fresh
  extracted source tree. ceres.patch records the exact library/CMake delta.

Algorithm
  Contiguous whole-row partitions are balanced by scalar block entries.
  Each partition accumulates both symmetric contributions into its own zeroed
  output buffer, retaining the original Ceres dense-block multiplication
  kernels. After a barrier, coordinates are reduced in fixed partition order
  into the caller's EXISTING y. Scratch memory belongs to each call; no mutable
  object/global buffers or atomics are used. Single-thread and small matrices
  retain the original serial path. Scheduling does not change summation order;
  parallel and serial summation orders differ only by ordinary roundoff.

Build environment
  Docker fisheye-slam; g++13.3, Eigen3.4, glog0.6, SuiteSparse7.6.
  Release, shared library, CUDA off, examples off, Eigen METIS off.
  BUILD_TESTING=ON provides ceres_static for the isolated kernel executable;
  only ceres and lamaria_matvec_benchmark targets are built.
  Compilation: 4 jobs, 4 CPU, 6 GiB container cap, no additional swap.
  Installation uses cmake --install with --prefix /work/build/.../install.

Validation and measured result
  build/ceres_lamaria_parallel/kernel_contracts.log:
    1,2,4,12 workers; diagonal/offdiagonal and mixed 2/3/6/15 dimensions;
    dense and unmodified serial oracles; nonzero initial y; repeated bitwise
    identical outputs; concurrent calls on the same matrix/context. All pass.
  build/ceres_lamaria_parallel/abi_report.json:
    All public headers/generated config match installed2.2.0; SONAME4;
    all201 installed strong public Ceres symbols remain present.
  build/ceres_lamaria_parallel/benchmark_results.json:
    Exact sequence sparsity, 18,343 pose blocks, 9,255,191 upper6x6 blocks,
    2,665,495,008 numeric bytes. Values are deterministic finite synthetic
    numbers; the diagonal blocks are symmetric. First fixed pose is retained
    conservatively in this benchmark pattern.
    9 alternating timings each, while existing visual refinement continued:
      serial median   0.270786257 s
      12-worker median0.081736965 s
      speedup         3.312898x
      peak RSS        3.985416 GiB (8 GiB cap)
      max relative numerical difference4.81368e-15; bitwise repeatable.
    This is kernel speedup, not measured global-iteration speedup. Graph
    assembly, ~909M point-induced pair visits, Jacobians and all other work
    remain. The later actual VI run is needed to measure total benefit.

Runtime deployment
  A separate solver must use an explicit private-library RPATH. Record the
  actual resolved libceres path and SHA in its execution manifest. Do not
  replace system libceres or rely on an unrecorded LD_LIBRARY_PATH. The
  factor/global fixture agent completed that isolated relink and its results:
    build/colmap_lamaria_ba_parallel/contracts.log (17 passed)
    build/colmap_lamaria_ba_parallel/end_to_end_explicit/results.json (3 passed)
    build/colmap_lamaria_ba_parallel/runtime_ldd.txt (private library resolved)
  Against the identical midpoint solver with system Ceres, maximum body-pose
  differences were 3.17e-15m visual, 2.04e-10m VI fixed, 1.58e-9m VI calibration.
  The parent solver workflow froze the validated bundle and selected the
  candidate for future VI arms; the active visual run remains unchanged.
