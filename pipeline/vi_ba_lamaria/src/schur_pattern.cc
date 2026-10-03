// Read-only exact reduced pose-block sparsity from COLMAP landmark tracks.
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
template<class T>T Read(std::istream& in){T v;in.read(reinterpret_cast<char*>(&v),sizeof(v));if(!in)throw std::runtime_error("Truncated COLMAP binary");return v;}
int main(int argc,char**argv)try{
  if(argc!=2 && argc!=3)throw std::runtime_error("Usage: schur_pattern MODEL [SORTED_UPPER_PAIRS_BIN]");
  const std::string root=argv[1];std::ifstream frames(root+"/frames.txt");
  if(!frames)throw std::runtime_error("Cannot read frames.txt");
  std::unordered_map<uint32_t,uint32_t> image_frame;std::string line;uint32_t n=0;
  while(std::getline(frames,line)){
    if(line.empty() || line[0]=='#')continue;
    std::istringstream row(line);uint32_t fid,rig,ndata;double v;
    row>>fid>>rig;for(int i=0;i<7;++i)row>>v;row>>ndata;
    for(uint32_t k=0;k<ndata;++k){std::string type;uint32_t sensor,image;row>>type>>sensor>>image;if(type!="CAMERA" || !row)throw std::runtime_error("Unexpected frame record");image_frame.emplace(image,n);}++n;
  }
  const uint64_t bits=uint64_t(n)*n;std::vector<uint64_t> pattern((bits+63)/64);
  auto mark=[&](uint32_t a,uint32_t b){if(a>b)std::swap(a,b);const uint64_t bit=uint64_t(a)*n+b;pattern[bit/64]|=uint64_t(1)<<(bit%64);};
  for(uint32_t i=0;i<n;++i)mark(i,i);
  std::ifstream points(root+"/points3D.bin",std::ios::binary);if(!points)throw std::runtime_error("Cannot read points3D.bin");
  const auto count=Read<uint64_t>(points);uint64_t observations=0,pairs=0;std::vector<uint32_t> track;
  for(uint64_t i=0;i<count;++i){
    points.ignore(43);const auto length=Read<uint64_t>(points);track.clear();track.reserve(length);
    for(uint64_t j=0;j<length;++j){const auto iid=Read<uint32_t>(points);Read<uint32_t>(points);track.push_back(image_frame.at(iid));}
    observations+=length;std::sort(track.begin(),track.end());track.erase(std::unique(track.begin(),track.end()),track.end());
    for(size_t j=0;j<track.size();++j)for(size_t k=j+1;k<track.size();++k){mark(track[j],track[k]);++pairs;}
  }
  uint64_t upper=0;for(auto word:pattern)upper+=__builtin_popcountll(word);
  if(argc==3){
    std::ofstream cache(argv[2],std::ios::binary);const uint64_t frames=n;
    cache.write(reinterpret_cast<const char*>(&frames),8);cache.write(reinterpret_cast<const char*>(&upper),8);
    for(uint64_t i=0;i<pattern.size();++i){uint64_t word=pattern[i];while(word){const uint64_t bit=i*64+__builtin_ctzll(word);const uint32_t row=bit/n,col=bit%n;cache.write(reinterpret_cast<const char*>(&row),4);cache.write(reinterpret_cast<const char*>(&col),4);word&=word-1;}}
    if(!cache)throw std::runtime_error("Cannot write sorted pair cache");
  }
  const uint64_t full=2*upper-n;
  std::cout<<"{\"frames\":"<<n<<",\"points\":"<<count<<",\"observations\":"<<observations
    <<",\"landmark_pair_visits\":"<<pairs<<",\"upper_pose_blocks_including_diagonal\":"<<upper
    <<",\"full_symmetric_pose_blocks\":"<<full<<",\"upper_block_values_bytes\":"<<upper*36*8
    <<",\"full_symmetric_values_bytes\":"<<full*36*8<<",\"full_scalar_csr_values_and_indices_bytes\":"<<full*36*12+(uint64_t(n)*6+1)*4
    <<",\"bitset_bytes\":"<<pattern.size()*8<<",\"constant_first_pose_included_conservatively\":true}\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
