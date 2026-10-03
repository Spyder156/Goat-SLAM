# Third-party software and attribution

This project builds on ORB-SLAM3 and retains the upstream authors, copyright
headers and component licenses. The project-specific native camera, rig,
tracking, initialization and calibration changes are recorded separately in the
source snapshots and `patches/`. This document identifies the components; it
does not replace their license texts or claim ownership of upstream work.

## ORB-SLAM3 and bundled libraries

| Component | Attribution / terms in the inspected source | License or attribution location |
|---|---|---|
| ORB-SLAM3 | Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel, Juan D. Tardós, University of Zaragoza; source headers specify GPL version 3 or later | `third_party/ORB_SLAM3/LICENSE`, `README.md`, and source headers |
| ORB-SLAM2 ancestry | Raúl Mur-Artal, José M. M. Montiel, Juan D. Tardós; attribution retained by ORB-SLAM3 | ORB-SLAM3 source headers and README |
| Bundled DBoW2 / DUtils | Dorian Gálvez-López; the ORB-SLAM2-derived distribution is identified as BSD in its README | `third_party/ORB_SLAM3/Thirdparty/DBoW2/README.txt`; full historical notice below |
| Bundled g2o subset | Rainer Kümmerle, Giorgio Grisetti, Hauke Strasdat, Kurt Konolige, Wolfram Burgard; BSD notice | `third_party/ORB_SLAM3/Thirdparty/g2o/license-bsd.txt` |
| Bundled Sophus | Hauke Strasdat, Steven Lovegrove; MIT | `third_party/ORB_SLAM3/Thirdparty/Sophus/LICENSE.txt` |
| ELSED | Iago Suárez, José M. Buenaposada, Luis Baumela; Apache 2.0 | `third_party/ELSED/LICENSE`, `README.md` |
| pybind11, if included with ELSED | Wenzel Jakob and contributors; BSD 3-Clause | `third_party/ELSED/pybind11/LICENSE` |
| Fisheye624 camera port | The native camera files identify a port from the Mecka Basalt `basalt-headers` implementation; the latter carries the BSD 3-Clause notice of Vladyslav Usenko and Nikolaus Demmel | Fisheye624 source comments; original Basalt notice below |

ORB-SLAM3 also records incorporated OpenCV ORB extraction, Vincent Lepetit's
EPnP, Steffen Urban's MLPnP, and public-domain bit-counting code in
`third_party/ORB_SLAM3/Dependencies.md`. Keep that file and the relevant source
notices with the source distribution.

Official origins:

- ORB-SLAM3: https://github.com/UZ-SLAMLab/ORB_SLAM3
- ORB-SLAM2: https://github.com/raulmur/ORB_SLAM2
- ELSED: https://github.com/iago-suarez/ELSED
- Basalt: https://gitlab.com/VladyslavUsenko/basalt

## Optional global refinement and runtime dependencies

The standalone solver in `pipeline/vi_ba_lamaria/` uses COLMAP scene/camera
structures and Ceres optimization. It is project code added alongside COLMAP;
it is not a claim that upstream COLMAP provides this visual-inertial solver.

| Component | Inspected license / use | Source of terms |
|---|---|---|
| COLMAP | New BSD / BSD 3-Clause; ETH Zurich and UNC Chapel Hill | `third_party/colmap_src/COPYING.txt`; https://github.com/colmap/colmap |
| Ceres Solver 2.2.0 | Main BSD notice; the release license additionally contains notices for bundled Apache and MIT components | https://github.com/ceres-solver/ceres-solver/blob/2.2.0/LICENSE |
| Eigen, OpenCV, Boost, Pangolin, SuiteSparse and other linked system libraries | Installed build/runtime dependencies, each with its own version-specific terms | Installed distribution copyright/license records and build manifests |
| Project Aria Tools | External VRS/calibration API used by the dataset preparation scripts | https://github.com/facebookresearch/projectaria_tools |
| LaMAria toolkit and dataset | External scoring/data inputs; their terms remain separate from this code | Obtain from their original distribution |
| ALIKED and optional learned matching packages | The validated runner consumes pre-extracted feature files; cached features and model weights are not made part of the software license by referencing them | Original model/package distribution and the feature-generation provenance |
| Rerun, NumPy, SciPy, OpenCV Python and plotting packages | External analysis/visualization dependencies | Installed package license metadata |

A reproducible optional VI-BA build needs the matching COLMAP revision and any
local modifications, the Ceres revision plus `patches/ceres_lamaria_parallel_2_2/`,
and the recorded toolchain. A saved executable does not replace those inputs.
Any redistributed binary should remain paired with its corresponding source,
build instructions and applicable component notices. A repository-level license
does not relicense independently licensed dependencies or datasets.

Unrelated research checkouts, downloaded papers, model weights, camera recordings,
feature caches, maps and large experiment outputs are separate from the source
release. Referencing a paper or optional package does not mean it is integrated
or that its artifacts are redistributed here.

## Historical DBoW2 notice

The inspected ORB-SLAM3 copy references `LICENSE.txt`, but that file is absent
from its bundled DBoW2 directory. The following notice is retained from the
original ORB-SLAM2 distribution identified by that directory's README:
https://github.com/raulmur/ORB_SLAM2/blob/master/Thirdparty/DBoW2/LICENSE.txt

This is the notice for the inherited distribution, not a substitution of the
current standalone DBoW2/DLib terms.

```text
DBoW2: bag-of-words library for C++ with generic descriptors

Copyright (c) 2015 Dorian Galvez-Lopez <http://doriangalvez.com> (Universidad de Zaragoza)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. Neither the name of copyright holders nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL COPYRIGHT HOLDERS OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

If you use it in an academic work, please cite:

  @ARTICLE{GalvezTRO12,
    author={G\'alvez-L\'opez, Dorian and Tard\'os, J. D.}, 
    journal={IEEE Transactions on Robotics},
    title={Bags of Binary Words for Fast Place Recognition in Image Sequences},
    year={2012},
    month={October},
    volume={28},
    number={5},
    pages={1188--1197},
    doi={10.1109/TRO.2012.2197158},
    ISSN={1552-3098}
  }
```

## Basalt headers notice

The following notice accompanies the documented Fisheye624 port.

```text
BSD 3-Clause License

Copyright (c) 2019, Vladyslav Usenko and Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```
