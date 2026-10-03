#!/usr/bin/env python3
"""Patch a private Ceres 2.2.0 staging tree; never edits vendor/system files."""
from pathlib import Path
import sys
root=Path(sys.argv[1]).resolve()
path=root/'internal/ceres/block_random_access_sparse_matrix.cc'
source=path.read_text()
source=source.replace('#include "ceres/parallel_vector_ops.h"','#include "ceres/parallel_vector_ops.h"\n#include "ceres/parallel_for.h"')
start=source.index('  for (int row_block_id = 0;',source.index('void BlockRandomAccessSparseMatrix::SymmetricRightMultiplyAndAccumulate'))
end=source.index('\n}\n',start)
original=source[start:end]
inner=original.replace('  for (int row_block_id = 0; row_block_id < num_blocks; ++row_block_id) {','    for (int row_block_id = begin; row_block_id < end; ++row_block_id) {').replace('y + row_block_pos','output + row_block_pos').replace('y + col_block_pos','output + col_block_pos')
replacement='''  // No mutable object scratch: separate concurrent multiplies remain safe.
  // Each partition owns a complete output because upper-triangle cells write
  // both row and column blocks. Fixed partitions/reduction order make worker
  // scheduling irrelevant to the floating-point result.
  const auto multiply_rows = [&](int begin, int end, double* output) {
'''+inner+'''
  };
  if (num_threads_ == 1 || num_blocks < 2 || bsm_->num_nonzeros() < 4096) {
    multiply_rows(0, num_blocks, y);
    return;
  }
  const int partitions = std::min(num_threads_, num_blocks);
  const int dimension = num_rows();
  std::vector<int64_t> cumulative(num_blocks + 1, 0);
  for (int row = 0; row < num_blocks; ++row) {
    cumulative[row + 1] = cumulative[row];
    if (!bs->rows[row].cells.empty()) {
      const auto& cell = bs->rows[row].cells.back();
      cumulative[row + 1] = int64_t(cell.position) +
          int64_t(blocks_[row].size) * blocks_[cell.block_id].size;
    }
  }
  std::vector<int> boundaries(partitions + 1, 0);
  boundaries.back() = num_blocks;
  for (int part = 1; part < partitions; ++part) {
    boundaries[part] = std::lower_bound(cumulative.begin(), cumulative.end(),
        cumulative.back() * part / partitions) - cumulative.begin();
  }
  std::vector<double> partial(size_t(partitions) * dimension, 0.0);
  ParallelFor(context_, 0, partitions, num_threads_, [&](int part) {
    multiply_rows(boundaries[part], boundaries[part + 1],
                  partial.data() + size_t(part) * dimension);
  });
  // This method accumulates into an arbitrary existing y, rather than clearing
  // it. Each final coordinate has one writer and a stable summation order.
  ParallelFor(context_, 0, dimension, num_threads_, [&](int index) {
    double total = y[index];
    for (int part = 0; part < partitions; ++part) {
      total += partial[size_t(part) * dimension + index];
    }
    y[index] = total;
  });'''
source=source[:start]+replacement+source[end:]
path.write_text(source)
cmake=root/'internal/ceres/CMakeLists.txt'
cmake.write_text(cmake.read_text()+'''
# Isolated LaMAria kernel contract/benchmark; not installed or exported.
add_executable(lamaria_matvec_benchmark lamaria_matvec_benchmark.cc)
target_include_directories(lamaria_matvec_benchmark PRIVATE ${Ceres_SOURCE_DIR}/internal)
target_link_libraries(lamaria_matvec_benchmark PRIVATE ceres_static ${CERES_LIBRARY_PRIVATE_DEPENDENCIES})
''')
(root/'internal/ceres/lamaria_matvec_benchmark.cc').write_text((Path(__file__).parent/'matvec_benchmark.cc').read_text())

config=root/"CMakeLists.txt"
text=config.read_text().replace("if (NOT SuiteSparse_Partition_FOUND)","# Match Ubuntu Ceres2.2.0 build feature configuration.\nset(SuiteSparse_Partition_FOUND FALSE)\nif (NOT SuiteSparse_Partition_FOUND)",1)
config.write_text(text)
