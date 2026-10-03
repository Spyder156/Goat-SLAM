#include "observation_metadata.h"
#include <iostream>
#include <map>
#include <cstring>

void Check(bool ok,const char* label){if(!ok)throw std::runtime_error(label);std::cout<<"PASS "<<label<<'\n';}
int main(int argc,char** argv) try {
  if(argc!=2)throw std::runtime_error("Expected existing small rig fixture model path");
  colmap::Reconstruction rec;rec.Read(argv[1]);
  const auto pid=rec.Points3D().begin()->first;
  const auto element=rec.Point3D(pid).track.Element(0);auto& im=rec.Image(element.image_id);
  const auto duplicate_index=im.NumPoints2D();const auto old_xy=im.Point2D(element.point2D_idx).xy;
  im.Points2D().emplace_back();im.Points2D().back().xy=old_xy+Eigen::Vector2d(.125,-.375);
  im.SetPoint3DForPoint2D(duplicate_index,pid);rec.Point3D(pid).track.AddElement(element.image_id,duplicate_index);
  im.Points2D().emplace_back();im.Points2D().back().xy=Eigen::Vector2d(-.25,401.125);
  const colmap::Reconstruction original=rec;
  std::map<colmap::point3D_t,const double*> point_addresses;
  std::map<colmap::frame_t,const double*> pose_addresses;
  for(const auto& entry:rec.Points3D())point_addresses[entry.first]=entry.second.xyz.data();
  for(const auto& entry:rec.Frames())pose_addresses[entry.first]=entry.second.RigFromWorld().params.data();
  const auto cache=std::filesystem::temp_directory_path()/ ("lamaria_metadata_contract_"+std::to_string(::getpid())+".cache");
  lamaria_ba::ObservationMetadata metadata(rec,cache);
  bool empty=true;for(const auto& entry:rec.Images())empty&=entry.second.Points2D().empty() && entry.second.NumPoints3D()==0;
  for(const auto& entry:rec.Points3D())empty&=entry.second.track.Length()==0;
  Check(empty && metadata.released_capacity_bytes>0,"nested observation arrays released with zero cached image association counts");
  auto& xyz=rec.Point3D(pid).xyz;xyz.x()+=.123;
  auto& pose=rec.Frame(im.FrameId()).RigFromWorld();pose.translation().y()+=.456;
  const auto changed_xyz=xyz;const auto changed_pose=pose.params;
  metadata.Restore(rec);
  bool exact=true;
  for(const auto& entry:original.Images()){
    const auto& a=entry.second;const auto& b=rec.Image(entry.first);
    exact&=a.NumPoints2D()==b.NumPoints2D() && a.NumPoints3D()==b.NumPoints3D();
    for(size_t i=0;i<a.NumPoints2D();++i)exact&=a.Point2D(i).point3D_id==b.Point2D(i).point3D_id && std::memcmp(a.Point2D(i).xy.data(),b.Point2D(i).xy.data(),2*sizeof(double))==0;
  }
  for(const auto& entry:original.Points3D())exact&=entry.second.track==rec.Point3D(entry.first).track;
  Check(exact && metadata.digest==metadata.restored_digest && metadata.restored,
        "all XY bits ownership counters unmatched features and same-image distinct track entries restored exactly");
  bool stable=true;for(const auto& entry:point_addresses)stable&=entry.second==rec.Point3D(entry.first).xyz.data();
  for(const auto& entry:pose_addresses)stable&=entry.second==rec.Frame(entry.first).RigFromWorld().params.data();
  Check(stable && (xyz-changed_xyz).norm()==0 && (pose.params-changed_pose).norm()==0,
        "all Ceres pose/XYZ addresses remain stable and optimized geometry survives metadata restoration");
  std::filesystem::remove(cache);
  colmap::Reconstruction corrupt=original;lamaria_ba::ObservationMetadata truncated(corrupt,cache);
  std::filesystem::resize_file(cache,17);bool rejected=false;try{truncated.Restore(corrupt);}catch(...){rejected=true;}
  Check(rejected,"truncated metadata cache fails explicitly before export");std::filesystem::remove(cache);
  std::cout<<"ALL METADATA CONTRACTS PASS\n";return 0;
} catch(const std::exception& e){std::cerr<<"METADATA_CONTRACT_FAIL "<<e.what()<<'\n';return 1;}
