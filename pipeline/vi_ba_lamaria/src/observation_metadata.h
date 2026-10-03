#pragma once
// Temporarily page out immutable observation metadata, never pose/XYZ storage.
#include "colmap/scene/reconstruction.h"
#include <openssl/evp.h>
#include <array>
#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <malloc.h>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <unistd.h>
#include <fcntl.h>

namespace lamaria_ba {
class ObservationMetadata {
 public:
  size_t image_count=0, point_count=0, feature_count=0, track_count=0;
  size_t released_capacity_bytes=0, max_image_features=0, max_track_length=0;
  std::string digest;
  std::string restored_digest;
  bool restored=false;

  ObservationMetadata(colmap::Reconstruction& rec,const std::filesystem::path& path):path_(path) {
    image_count=rec.NumImages();point_count=rec.NumPoints3D();
    Stream out(path_,true);
    out.Put(uint64_t(0x314154454d424c));out.Put(uint64_t(image_count));out.Put(uint64_t(point_count));
    for(const auto& entry:rec.Images()) {
      auto& im=rec.Image(entry.first);auto& features=im.Points2D();
      out.Put(uint64_t(entry.first));out.Put(uint64_t(features.size()));out.Put(uint64_t(im.NumPoints3D()));
      feature_count+=features.size();max_image_features=std::max(max_image_features,features.size());
      released_capacity_bytes+=features.capacity()*sizeof(colmap::Point2D);
      for(const auto& feature:features){out.Put(feature.xy.x());out.Put(feature.xy.y());out.Put(uint64_t(feature.point3D_id));}
      std::vector<colmap::Point2D>().swap(features);
      im.SetPoints2D(std::vector<colmap::Point2D>{});
    }
    for(const auto& entry:rec.Points3D()) {
      auto& track=rec.Point3D(entry.first).track;
      out.Put(uint64_t(entry.first));out.Put(uint64_t(track.Length()));
      track_count+=track.Length();max_track_length=std::max(max_track_length,track.Length());
      released_capacity_bytes+=track.Elements().capacity()*sizeof(colmap::TrackElement);
      for(const auto& el:track.Elements()){out.Put(uint32_t(el.image_id));out.Put(uint32_t(el.point2D_idx));}
      track.SetElements({});
    }
    digest=out.Finish();
    // Flush this owned cache, then discard only its clean page-cache pages.
    // This avoids exchanging freed anonymous memory for dirty file pages.
    if(std::fflush(out.file.get())!=0 || ::fsync(::fileno(out.file.get()))!=0)
      throw std::runtime_error("Cannot flush observation metadata cache");
    ::posix_fadvise(::fileno(out.file.get()),0,0,POSIX_FADV_DONTNEED);
    ::malloc_trim(0);
  }

  void Restore(colmap::Reconstruction& rec) {
    if(restored)throw std::runtime_error("Observation metadata restored twice");
    if(rec.NumImages()!=image_count || rec.NumPoints3D()!=point_count)
      throw std::runtime_error("Reconstruction membership changed while observations were released");
    Stream in(path_,false);
    if(in.Get<uint64_t>()!=0x314154454d424c || in.Get<uint64_t>()!=image_count || in.Get<uint64_t>()!=point_count)
      throw std::runtime_error("Observation metadata header changed");
    size_t features_read=0,tracks_read=0;
    for(size_t i=0;i<image_count;++i) {
      const auto id=in.Get<uint64_t>(),count=in.Get<uint64_t>(),associated=in.Get<uint64_t>();
      if(id>std::numeric_limits<colmap::image_t>::max() || !rec.ExistsImage(id) || count>max_image_features || associated>count)
        throw std::runtime_error("Invalid observation metadata image record");
      auto& im=rec.Image(id);
      if(!im.Points2D().empty())throw std::runtime_error("Image feature storage was replaced during solve");
      std::vector<colmap::Point2D> features(count);
      for(auto& feature:features){feature.xy.x()=in.Get<double>();feature.xy.y()=in.Get<double>();feature.point3D_id=in.Get<uint64_t>();}
      im.SetPoints2D(std::move(features));
      if(im.NumPoints3D()!=associated)throw std::runtime_error("Restored image association count differs");
      features_read+=count;
    }
    for(size_t i=0;i<point_count;++i) {
      const auto id=in.Get<uint64_t>(),count=in.Get<uint64_t>();
      if(!rec.ExistsPoint3D(id) || count>max_track_length)throw std::runtime_error("Invalid observation metadata landmark record");
      auto& track=rec.Point3D(id).track;
      if(track.Length())throw std::runtime_error("Landmark tracks changed during solve");
      std::vector<colmap::TrackElement> elements;elements.reserve(count);
      for(size_t j=0;j<count;++j) {
        const auto image=in.Get<uint32_t>(),index=in.Get<uint32_t>();
        if(!rec.ExistsImage(image) || index>=rec.Image(image).NumPoints2D() || rec.Image(image).Point2D(index).point3D_id!=id)
          throw std::runtime_error("Restored track has inconsistent reciprocal image ownership");
        elements.emplace_back(image,index);
      }
      track.SetElements(std::move(elements));tracks_read+=count;
    }
    if(features_read!=feature_count || tracks_read!=track_count || in.Finish()!=digest || std::fgetc(in.file.get())!=EOF)
      throw std::runtime_error("Observation metadata exact restoration digest/count mismatch");
    restored_digest=MemoryDigest(rec);
    if(restored_digest!=digest)throw std::runtime_error("Restored in-memory metadata digest differs from original");
    restored=true;
    ::posix_fadvise(::fileno(in.file.get()),0,0,POSIX_FADV_DONTNEED);
  }

 private:
  struct Stream {
    std::unique_ptr<FILE,decltype(&std::fclose)> file{nullptr,&std::fclose};
    std::unique_ptr<EVP_MD_CTX,decltype(&EVP_MD_CTX_free)> hash{EVP_MD_CTX_new(),&EVP_MD_CTX_free};
    Stream() {
      if(!hash || EVP_DigestInit_ex(hash.get(),EVP_sha256(),nullptr)!=1)
        throw std::runtime_error("Cannot initialize observation metadata digest");
    }
    Stream(const std::filesystem::path& path,bool write):Stream() {
      file.reset(std::fopen(path.c_str(),write?"wb":"rb"));
      if(!file)
        throw std::runtime_error("Cannot open observation metadata stream");
    }
    template<class T> void Put(T value) {
      if((file && std::fwrite(&value,sizeof(T),1,file.get())!=1) || EVP_DigestUpdate(hash.get(),&value,sizeof(T))!=1)
        throw std::runtime_error("Cannot write observation metadata");
    }
    template<class T> T Get() {
      T value;
      if(std::fread(&value,sizeof(T),1,file.get())!=1 || EVP_DigestUpdate(hash.get(),&value,sizeof(T))!=1)
        throw std::runtime_error("Truncated observation metadata");
      return value;
    }
    std::string Finish() {
      unsigned char bytes[EVP_MAX_MD_SIZE];unsigned int size=0;
      if(EVP_DigestFinal_ex(hash.get(),bytes,&size)!=1)throw std::runtime_error("Observation metadata digest failed");
      std::ostringstream text;text<<std::hex<<std::setfill('0');
      for(unsigned int i=0;i<size;++i)text<<std::setw(2)<<unsigned(bytes[i]);return text.str();
    }
  };
  static std::string MemoryDigest(const colmap::Reconstruction& rec) {
    Stream hash;
    hash.Put(uint64_t(0x314154454d424c));hash.Put(uint64_t(rec.NumImages()));hash.Put(uint64_t(rec.NumPoints3D()));
    for(const auto& entry:rec.Images()) {
      const auto& im=entry.second;
      hash.Put(uint64_t(entry.first));hash.Put(uint64_t(im.NumPoints2D()));hash.Put(uint64_t(im.NumPoints3D()));
      for(const auto& feature:im.Points2D()){hash.Put(feature.xy.x());hash.Put(feature.xy.y());hash.Put(uint64_t(feature.point3D_id));}
    }
    for(const auto& entry:rec.Points3D()) {
      hash.Put(uint64_t(entry.first));hash.Put(uint64_t(entry.second.track.Length()));
      for(const auto& el:entry.second.track.Elements()){hash.Put(uint32_t(el.image_id));hash.Put(uint32_t(el.point2D_idx));}
    }
    return hash.Finish();
  }
  std::filesystem::path path_;
};
} // namespace lamaria_ba
