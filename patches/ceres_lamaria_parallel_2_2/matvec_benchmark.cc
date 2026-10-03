#include "ceres/block_random_access_sparse_matrix.h"
#include "ceres/context_impl.h"
#include "ceres/small_blas.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <set>
#include <stdexcept>
#include <sys/resource.h>
#include <vector>

namespace ci=ceres::internal;
using Matrix=ci::BlockRandomAccessSparseMatrix;
using Clock=std::chrono::steady_clock;
void Require(bool pass,const char* text){if(!pass)throw std::runtime_error(text);}
void Fill(Matrix& m) {
  auto* sparse=m.mutable_matrix();auto* values=sparse->mutable_values();const auto* bs=sparse->block_structure();
  for(int r=0;r<int(bs->rows.size());++r)for(const auto& cell:bs->rows[r].cells){
    const int rows=bs->rows[r].block.size,cols=bs->cols[cell.block_id].size;
    for(int i=0;i<rows;++i)for(int j=0;j<cols;++j){
      const int ii=(r==cell.block_id)?std::min(i,j):i,jj=(r==cell.block_id)?std::max(i,j):j;
      values[cell.position+i*cols+j]=double(int64_t((uint64_t(r)*31+cell.block_id*17+ii*7+jj*13)%257)-128)*.0001;
    }
  }
}
// Unmodified Ceres2.2 serial algorithm, on the SAME matrix values/layout.
void Serial(const Matrix& m,const double* x,double* y){
  const auto* bs=m.matrix()->block_structure();const auto* values=m.matrix()->values();
  for(int r=0;r<int(bs->rows.size());++r){const auto& row=bs->rows[r];
    for(const auto& c:row.cells){const auto& col=bs->cols[c.block_id];
      ci::MatrixVectorMultiply<Eigen::Dynamic,Eigen::Dynamic,1>(values+c.position,row.block.size,col.size,x+col.position,y+row.block.position);
      if(c.block_id!=r)ci::MatrixTransposeVectorMultiply<Eigen::Dynamic,Eigen::Dynamic,1>(values+c.position,row.block.size,col.size,x+row.block.position,y+col.position);
    }
  }
}
double Error(const std::vector<double>& a,const std::vector<double>& b){double num=0,den=0;for(size_t i=0;i<a.size();++i){num=std::max(num,std::abs(a[i]-b[i]));den=std::max(den,std::abs(a[i]));}return num/std::max(1.,den);}
void Contract(int threads,bool small){
  ci::ContextImpl context;context.EnsureMinimumThreads(threads);
  std::vector<ci::Block> blocks;const int dims[]={6,3,6,2,15};int n=0;
  for(int i=0;i<(small?1:40);++i){blocks.emplace_back(dims[i%5],n);n+=dims[i%5];}
  std::set<std::pair<int,int>> pairs;
  for(int i=0;i<int(blocks.size());++i)for(int j=i;j<int(blocks.size());++j)if(i==j||(i+j)%3)pairs.emplace(i,j);
  Matrix m(blocks,pairs,&context,threads);Fill(m);
  std::vector<double> x(n),initial(n),reference(n),actual(n);
  for(int i=0;i<n;++i){x[i]=std::sin(i*.13);initial[i]=.25+std::cos(i*.19);}
  reference=initial;Serial(m,x.data(),reference.data());actual=initial;m.SymmetricRightMultiplyAndAccumulate(x.data(),actual.data());
  Require(Error(reference,actual)<1e-12,"mixed blocks/nonzero accumulation differs from serial");
  // A separate dense oracle detects matching mistakes in both sparse loops.
  ceres::Matrix dense;m.matrix()->ToDenseMatrix(&dense);
  Eigen::Map<const Eigen::VectorXd> X(x.data(),n),Y(initial.data(),n);
  const Eigen::VectorXd expected=Y+dense.selfadjointView<Eigen::Upper>()*X;
  std::vector<double> oracle(expected.data(),expected.data()+n);
  Require(Error(oracle,actual)<1e-12,"mixed blocks differs from dense oracle");
  for(int i=0;i<4;++i){auto repeat=initial;m.SymmetricRightMultiplyAndAccumulate(x.data(),repeat.data());Require(repeat==actual,"parallel output not bitwise repeatable");}
  std::vector<std::future<bool>> futures;
  for(int call=0;call<3;++call)futures.emplace_back(std::async(std::launch::async,[&,call]{auto in=x,out=initial,ref=initial;for(auto& v:in)v*=1.+call*.37;Serial(m,in.data(),ref.data());m.SymmetricRightMultiplyAndAccumulate(in.data(),out.data());return Error(ref,out)<1e-12;}));
  for(auto& f:futures)Require(f.get(),"concurrent matvec calls corrupt output");
  std::cout<<"CONTRACT threads="<<threads<<" small="<<small<<" scalars="<<n<<" pass=1\n";
}
int main(int argc,char** argv)try{
  std::cout<<std::setprecision(17);
  if(argc==1){for(int t:{1,2,4,12}){Contract(t,true);Contract(t,false);}std::cout<<"ALL_KERNEL_CONTRACTS_PASS\n";return 0;}
  Require(argc>=3,"usage: [pairs.bin threads [repetitions]]");const int threads=std::stoi(argv[2]),reps=argc>3?std::stoi(argv[3]):7;
  std::ifstream f(argv[1],std::ios::binary);uint64_t count=0,pair_count=0;f.read(reinterpret_cast<char*>(&count),8);f.read(reinterpret_cast<char*>(&pair_count),8);Require(bool(f)&&count>0&&count<100000,"invalid pattern header");
  ci::ContextImpl context;context.EnsureMinimumThreads(threads);std::vector<ci::Block> blocks;for(int i=0;i<int(count);++i)blocks.emplace_back(6,6*i);
  std::set<std::pair<int,int>> pairs;for(uint64_t i=0;i<pair_count;++i){uint32_t rc[2];f.read(reinterpret_cast<char*>(rc),8);Require(bool(f)&&rc[0]<=rc[1]&&rc[1]<count,"invalid pattern pair");pairs.emplace_hint(pairs.end(),rc[0],rc[1]);}
  Matrix m(blocks,pairs,&context,threads);pairs.clear();Fill(m);const int n=m.num_rows();
  std::vector<double> x(n),initial(n),expected(n),actual(n);for(int i=0;i<n;++i){x[i]=std::sin(i*.13);initial[i]=.25+std::cos(i*.19);}
  expected=initial;Serial(m,x.data(),expected.data());actual=initial;m.SymmetricRightMultiplyAndAccumulate(x.data(),actual.data());const double error=Error(expected,actual);Require(error<1e-11,"large matrix serial/parallel mismatch");
  auto repeated=initial;m.SymmetricRightMultiplyAndAccumulate(x.data(),repeated.data());Require(repeated==actual,"large matrix not repeatable");
  std::vector<double> serial,parallel;double checksum=0;
  for(int r=0;r<reps;++r){
    // Alternating order avoids systematically favouring one cache/CPU state.
    for(int phase=0;phase<2;++phase){bool par=(r+phase)%2;auto y=initial;auto begin=Clock::now();if(par)m.SymmetricRightMultiplyAndAccumulate(x.data(),y.data());else Serial(m,x.data(),y.data());double seconds=std::chrono::duration<double>(Clock::now()-begin).count();(par?parallel:serial).push_back(seconds);checksum+=y[(r*97)%n];}
  }
  std::sort(serial.begin(),serial.end());std::sort(parallel.begin(),parallel.end());rusage usage{};getrusage(RUSAGE_SELF,&usage);
  std::cout<<"{\"threads\":"<<threads<<",\"blocks\":"<<count<<",\"upper_pairs\":"<<pair_count<<",\"scalars\":"<<n<<",\"matrix_values_bytes\":"<<size_t(m.matrix()->num_nonzeros())*8<<",\"repetitions\":"<<reps<<",\"serial_median_seconds\":"<<serial[serial.size()/2]<<",\"parallel_median_seconds\":"<<parallel[parallel.size()/2]<<",\"speedup\":"<<serial[serial.size()/2]/parallel[parallel.size()/2]<<",\"maximum_relative_error\":"<<error<<",\"bitwise_repeatable\":true,\"peak_rss_kib\":"<<usage.ru_maxrss<<",\"checksum\":"<<checksum<<"}\n";
  return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
