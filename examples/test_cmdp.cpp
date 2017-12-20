#include "../include/CMDParser.hpp"

#include <zmq.hpp>
#include <iostream>

#include "../include/VariantVec.hpp"
#include "../include/PointCloudGridEncoder.hpp"

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

    PointCloud<Vec<float>, Vec<float>> pc(BoundingBox(Vec<float>(-1.01f,-1.01f,-1.01f), Vec<float>(1.01f,1.01f,1.01f)));
    for(float x = -1.0f; x < 1.0; x += 0.035) {
        for(float y = -1.0f; y < 1.0; y += 0.035) {
            for(float z = -1.0f; z < 1.0; z += 0.035) {
                pc.points.emplace_back(x,y,z);
                pc.colors.emplace_back((x+1)/2.0f,(y+1)/2.0f,(z+1)/2.0f);
            }
        }
    }

    std::cout << "point_cloud:" << std::endl;
    std::cout << " > size:" << pc.size() << "\n";

    //// ENCODING

    PointCloudGridEncoder encoder;
    auto start = std::chrono::steady_clock::now();
    zmq::message_t msg = encoder.encode<uint8_t, uint8_t>(&pc, Vec8(4,4,4));
    auto end = std::chrono::steady_clock::now();

    unsigned elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    std::cout << "Encoding took " << elapsed << "ms.\n";
    int size_bytes = msg.size();
    int size_bit = size_bytes * 8;
    float mbit = size_bit / 1000000.0f;
    std::cout << "Message Size\n"
              << "  > bytes " << size_bytes << "\n"
              << "  > mbit " << mbit << "\n";

    //// DECODING

    start = std::chrono::steady_clock::now();
    bool success = encoder.decode(msg, &pc);
    end = std::chrono::steady_clock::now();

    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    std::cout << "Decoding took " << elapsed << "ms.\n";
    if(success)
        std::cout << "  > success: YES\n";
    else
        std::cout << "  > success: NO\n";
    /*
    unsigned tick = 0;
    while(true){

        zmq::message_t zmqm(sizeof(unsigned));

        memcpy( (unsigned char* ) zmqm.data(), (const unsigned char*) &tick, sizeof(unsigned));
        socket.send(zmqm);

        //std::cout << "sending: " << tick << std::endl;

        ++tick;
    }
    */

    return 0;
}


