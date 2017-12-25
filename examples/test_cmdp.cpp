#include "../include/Measurement.hpp"
#include "../include/CMDParser.hpp"

#include <zmq.hpp>

#include "../include/PointCloudGridEncoder.hpp"
#include "../include/BitVec.hpp"

int main(int argc, char* argv[]){
    /*
    CMDParser p("socket");
    p.init(argc,argv);

    std::string socket_name(p.getArgs()[0]);

    zmq::context_t ctx(1); // means single threaded
    zmq::socket_t  socket(ctx, ZMQ_PUB); // means a publisher

    uint32_t hwm = 1;
    socket.setsockopt(ZMQ_SNDHWM,&hwm, sizeof(hwm));

    std::string endpoint("tcp://" + socket_name);
    socket.bind(endpoint.c_str());
    */
    BitVecArray<5,5,5> arr;
    arr.data.emplace_back(2,5,3);
    arr.data.emplace_back(4,2,1);

    std::cout << "VEC BEFORE PACK\n";
    std::cout << "  > " << arr.data[0].x.to_ulong() << "," << arr.data[0].y.to_ulong() << "," << arr.data[0].z.to_ulong() << std::endl;
    std::cout << "  > " << arr.data[1].x.to_ulong() << "," << arr.data[1].y.to_ulong() << "," << arr.data[1].z.to_ulong() << std::endl;

    arr.calcPackedData();
    arr.fromPackedData(1);

    std::cout << "VEC AFTER PACK\n";
    std::cout << "  > " << arr.data[0].x.to_ulong() << "," << arr.data[0].y.to_ulong() << "," << arr.data[0].z.to_ulong() << std::endl;
    std::cout << "  > " << arr.data[1].x.to_ulong() << "," << arr.data[1].y.to_ulong() << "," << arr.data[1].z.to_ulong() << std::endl;

    Measure t;

    PointCloud<Vec<float>, Vec<float>> pc(BoundingBox(Vec<float>(-1.01f,-1.01f,-1.01f), Vec<float>(1.01f,1.01f,1.01f)));
    for(float x = -1.0f; x < 1.0; x += 0.5) {
        for(float y = -1.0f; y < 1.0; y += 0.5) {
            for(float z = -1.0f; z < 1.0; z += 0.5) {
                pc.points.emplace_back(x,y,z);
                pc.colors.emplace_back((x+1)/2.0f,(y+1)/2.0f,(z+1)/2.0f);
            }
        }
    }

    std::cout << "POINT CLOUD" << std::endl;
    std::cout << "  > size " << pc.size() << "\n";

    //// ENCODING

    PointCloudGridEncoder encoder;
    t.startWatch();
    zmq::message_t msg = encoder.encode<uint8_t, uint16_t>(&pc, Vec8(8,8,8));
    
    std::cout << "ENCODING DONE in " << t.stopWatch() << "ms.\n";
    auto size_bytes = static_cast<int>(msg.size());
    int size_bit = size_bytes * 8;
    float mbit = size_bit / 1000000.0f;

    std::cout << "  > Message Size\n"
              << "    > bytes " << size_bytes << "\n"
              << "    > mbit " << mbit << "\n";

    //// DECODING

    PointCloud<Vec<float>, Vec<float>> pc2;
    t.startWatch();
    bool success = encoder.decode(msg, &pc2);
    std::cout << "DECODING DONE in " << t.stopWatch() << "ms.\n";

    if(success)
        std::cout << "  > success: YES\n";
    else
        std::cout << "  > success: NO\n";


    t.startWatch();
    std::cout << "  > MSE " << t.meanSquaredErrorPC(pc, pc2) << std::endl;
    std::cout << "    > took " << t.stopWatch() << "ms" << std::endl;

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
