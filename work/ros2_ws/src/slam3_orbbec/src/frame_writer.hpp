#pragma once
#include <opencv2/imgcodecs.hpp>
#include <filesystem>
#include <fstream>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <iomanip>

// Bounded non-blocking producer; all PNG encoding and disk writes run here.
class FrameWriter {
 public:
  struct Job { std::string key; double timestamp; cv::Mat rgb,depth; };
  FrameWriter(std::string root,size_t capacity):root_(std::move(root)),capacity_(capacity) {
    if(!capacity_) throw std::runtime_error("writer queue capacity must be positive");
    std::filesystem::create_directories(root_+"/rgb");
    std::filesystem::create_directories(root_+"/depth");
    manifest_.open(root_+"/frames.txt");
    if(!manifest_) throw std::runtime_error("Cannot create frames.txt");
    manifest_ << "# timestamp rgb_path depth_path; rectified uint16 millimetres\n";
    thread_=std::thread([this]{work();});
  }
  ~FrameWriter(){close();}
  bool submit(Job j) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(stop_ || jobs_.size()>=capacity_) {++dropped; return false;}
    jobs_.push_back(std::move(j)); ready_.notify_one(); return true;
  }
  void drain() {
    std::unique_lock<std::mutex> lock(mutex_);
    drained_.wait(lock,[this]{return jobs_.empty()&&!busy_;});
  }
  void close() {
    {std::lock_guard<std::mutex> lock(mutex_);stop_=true;}
    ready_.notify_all();
    if(thread_.joinable()) thread_.join();
    manifest_.close();
  }
  std::string error(){std::lock_guard<std::mutex> lock(mutex_);return error_;}
  std::atomic<size_t> dropped{0},written{0},failed{0};
 private:
  void work() {
    for(;;) {
      Job j;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        ready_.wait(lock,[this]{return stop_||!jobs_.empty();});
        if(jobs_.empty()) break;
        j=std::move(jobs_.front());jobs_.pop_front();busy_=true;
      }
      try {
        std::string rgb="rgb/"+j.key+".png",depth="depth/"+j.key+".png";
        std::string rt=root_+"/rgb/"+j.key+".tmp.png",dt=root_+"/depth/"+j.key+".tmp.png";
        const std::vector<int> opts{cv::IMWRITE_PNG_COMPRESSION,1};
        if(!cv::imwrite(rt,j.rgb,opts)||!cv::imwrite(dt,j.depth,opts)) throw std::runtime_error("PNG write failed");
        std::filesystem::rename(rt,root_+"/"+rgb);
        std::filesystem::rename(dt,root_+"/"+depth);
        manifest_<<std::fixed<<std::setprecision(9)<<j.timestamp<<' '<<rgb<<' '<<depth<<'\n';
        manifest_.flush();
        if(!manifest_) throw std::runtime_error("RGB-D manifest write failed");
        ++written;
      } catch(const std::exception& e) {
        ++failed;std::lock_guard<std::mutex> lock(mutex_);error_=e.what();
      }
      {std::lock_guard<std::mutex> lock(mutex_);busy_=false;}
      drained_.notify_all();
    }
    drained_.notify_all();
  }
  std::string root_,error_;size_t capacity_;std::ofstream manifest_;
  std::deque<Job> jobs_;std::mutex mutex_;std::condition_variable ready_,drained_;
  bool stop_=false,busy_=false;std::thread thread_;
};
