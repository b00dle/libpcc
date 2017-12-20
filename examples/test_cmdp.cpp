#include "CMDParser.hpp"
#include "Measurement.hpp"
#include "PointCloud.hpp"

#include <zmq.hpp>

#include <iostream>

int main(int argc, char* argv[]){

  CMDParser p("socket");
  p.init(argc,argv);

  std::string socket_name(p.getArgs()[0]);

  zmq::context_t ctx(1); // means single threaded
  zmq::socket_t  socket(ctx, ZMQ_PUB); // means a publisher

  uint32_t hwm = 1;
  socket.setsockopt(ZMQ_SNDHWM,&hwm, sizeof(hwm));

  std::string endpoint("tcp://" + socket_name);
  socket.bind(endpoint.c_str());


  PointCloud<Vec32, Vec32> pc(BoundingBox(-1.01, 1.01, -1.01, 1.01, -1.01, 1.01));
  for(float x = -1.0; x < 1.0; x += 0.05) {
      for(float y = -1.0; y < 1.0; y += 0.01) {
          for(float z = -1.0; z < 1.0; z += 0.05) {
              pc.points.push_back(Vec32(x,y,z));
              pc.colors.push_back(Vec32((x+1)/2.0f,(y+1)/2.0f,(z+1)/2.0f));
          }
      }
  }

  PointCloud<Vec32, Vec32> pc2(BoundingBox(-1.01, 1.01, -1.01, 1.01, -1.01, 1.01));
  for(float x = -1.0; x < 1.0; x += 0.05) {
      for(float y = -1.0; y < 1.0; y += 0.05) {
          for(float z = -1.0; z < 1.0; z += 0.01) {
              pc2.points.push_back(Vec32(x,y,z));
              pc2.colors.push_back(Vec32((x+1)/2.0f,(y+1)/2.0f,(z+1)/2.0f));
          }
      }
  }

  PointCloud<Vec32, Vec32> pc3(BoundingBox(-1.01, 1.01, -1.01, 1.01, -1.01, 1.01));
  PointCloud<Vec32, Vec32> pc4(BoundingBox(-1.01, 1.01, -1.01, 1.01, -1.01, 1.01));
  pc3.points.push_back(Vec32(1, 1, 1));
  pc3.points.push_back(Vec32(1, 1, 0));
  pc3.points.push_back(Vec32(1, 0, 0));
  pc3.points.push_back(Vec32(0, 0, 0));
  pc4.points.push_back(Vec32(-1, -1, -1));
  pc4.points.push_back(Vec32(-1, -1, 0));
  pc4.points.push_back(Vec32(-1, 0, 0));
  pc4.points.push_back(Vec32(0, 0, 0));


  Measure t;
  t.startWatch();
  std::cout << t.meanSquaredErrorPC(pc, pc2) << std::endl;
  t.printTimeSpan(t.stopWatch());

  unsigned tick = 0;
  // while(true){

    // auto t1 = t.createTimeStamp();
    // t.startWatch();

    // for(unsigned int i = 0; i < 1000000; ++i) {
    //   std::cout << "----------" << std::endl;
    // }

    // auto t2 = t.createTimeStamp();
    // t.printTimeStamp(t1);
    // t.printTimeStamp(t2);
    // auto time_span = t.calcTimeSpan(t1, t2);
    // t.printTimeSpan(time_span);
    // std::cout << "StopWatch: " << std::endl;
    // t.printTimeSpan(t.stopWatch());
    // break;
    //
    // auto test = t.Measure::setTimeStamp();
    // t.printTimeStamp(test);
    //
    // zmq::message_t zmqm(sizeof(unsigned));
    // memcpy( (unsigned char* ) zmqm.data(), (const unsigned char*) &tick, sizeof(unsigned));
    // socket.send(zmqm);
    //
    // std::cout << "sending: " << tick << std::endl;
    //
    // ++tick;
  // }

  return 0;
}
