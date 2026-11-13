#include "../../work/ros2_ws/src/slam3_orbbec/src/frame_writer.hpp"
#include <iostream>
#include <cassert>
int main(int argc,char** argv) {
  std::filesystem::create_directories(argv[1]);
  FrameWriter writer(argv[1],1);
  cv::Mat color(800,1280,CV_8UC3);cv::randu(color,0,255);
  cv::Mat depth(800,1280,CV_16U,cv::Scalar(1000));
  for(int i=0;i<100;i++) writer.submit({std::to_string(i),double(i),color,depth});
  writer.drain();writer.close();
  assert(writer.dropped>0);assert(writer.written+writer.dropped==100);assert(writer.failed==0);
  std::cout<<"written="<<writer.written<<" dropped="<<writer.dropped<<"; bounded queue and drain OK\n";
}
