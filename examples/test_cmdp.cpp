#include "../include/Measurement.hpp"
#include "../include/CMDParser.hpp"

#include <zmq.hpp>

#include "../include/PointCloudGridEncoder.hpp"
#include "../include/BinaryFile.hpp"

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
    Measure t;

    PointCloud<Vec<float>, Vec<float>> pc(BoundingBox(Vec<float>(-1.01f,-1.01f,-1.01f), Vec<float>(1.01f,1.01f,1.01f)));
    UncompressedPointCloud pc_uncomp(BoundingBox(Vec<float>(-1.01f,-1.01f,-1.01f), Vec<float>(1.01f,1.01f,1.01f)));
    for(float x = -1.0f; x < 1.0; x += 0.04) {
        for(float y = -1.0f; y < 1.0; y += 0.04) {
            for(float z = -1.0f; z < 1.0; z += 0.04) {
                pc.points.emplace_back(x,y,z);
                pc.colors.emplace_back((x+1)/2.0f,(y+1)/2.0f,(z+1)/2.0f);
                pc_uncomp.data.push_back(UncompressedVoxel());
                pc_uncomp.data.back().pos[0] = x;
                pc_uncomp.data.back().pos[1] = y;
                pc_uncomp.data.back().pos[2] = z;
                pc_uncomp.data.back().color_rgba[0] = static_cast<unsigned char>(pc.colors.back().x*255.0f);
                pc_uncomp.data.back().color_rgba[1] = static_cast<unsigned char>(pc.colors.back().y*255.0f);
                pc_uncomp.data.back().color_rgba[2] = static_cast<unsigned char>(pc.colors.back().z*255.0f);
                pc_uncomp.data.back().color_rgba[3] = 255;
            }
        }
    }

    std::cout << "POINT CLOUD" << std::endl;
    std::cout << "  > size " << pc.size() << "\n";

    //// ENCODING

    PointCloudGridEncoder encoder;
    encoder.settings.grid_precision = GridPrecisionDescriptor(
        Vec8(4,4,4), // dimensions
        Vec<BitCount>(BIT_4,BIT_4,BIT_4), // default point encoding
        Vec<BitCount>(BIT_8,BIT_8,BIT_8)  // default color encoding
    );
    encoder.settings.num_threads = 24;

    t.startWatch();
    zmq::message_t msg = encoder.encode(&pc_uncomp);

    std::cout << "ENCODING DONE in " << t.stopWatch() << "ms.\n";
    auto size_bytes = static_cast<int>(msg.size());
    int size_bit = size_bytes * 8;
    float mbit = size_bit / 1000000.0f;

    std::cout << "  > Message Size\n"
              << "    > bytes " << size_bytes << "\n"
              << "    > mbit " << mbit << "\n";

    //// TESTING FILE READ/WRITE

    /*
    BinaryFile f(msg);
    if(f.write("./test_pc_grid.txt")) {
        std::cout << "WRITE TO FILE done.\n";
        if(f.read("./test_pc_grid.txt")) {
            f.copy((char*) msg.data());
            std::cout << "READ FROM FILE done.\n";
        }
        else {
            std::cout << "READ FROM FILE failed.\n";
        }
    }
    else {
        std::cout << "WRITE TO FILE failed.\n";
    }
    */

    //// DECODING

    PointCloud<Vec<float>, Vec<float>> pc2;
    t.startWatch();
    //zmq::message_t msg2 = f.get();
    bool success = encoder.decode(msg, &pc2);
    std::cout << "DECODING DONE in " << t.stopWatch() << "ms.\n";
    std::cout << "  > size " << pc2.size() << "\n";
    if(success)
        std::cout << "  > success: YES\n";
    else
        std::cout << "  > success: NO\n";

    /*
    t.startWatch();
    std::cout << "  > MSE " << t.meanSquaredErrorPC(pc, pc2) << std::endl;
    std::cout << "    > took " << t.stopWatch() << "ms" << std::endl;
    */
    
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
