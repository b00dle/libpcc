#include <CMDParser.hpp>
#include <Measurement.hpp>

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

  Measure t;

  unsigned tick = 0;
  while(true){

    auto test = t.Measure::setTimeStamp();
    t.printTimeStamp(test);

    zmq::message_t zmqm(sizeof(unsigned));
    memcpy( (unsigned char* ) zmqm.data(), (const unsigned char*) &tick, sizeof(unsigned));
    socket.send(zmqm);

    std::cout << "sending: " << tick << std::endl;

    ++tick;
  }

  return 0;
}
