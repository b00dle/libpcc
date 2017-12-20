#include "../include/Measurement.hpp"
#include "../include/CMDParser.hpp"

#include <zmq.hpp>

#include "../include/PointCloudGridEncoder.hpp"

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
    
    PointCloud<Vec<float>, Vec<float>> pc(BoundingBox(Vec<float>(-1.01f,-1.01f,-1.01f), Vec<float>(1.01f,1.01f,1.01f)));
    for(float x = -1.0f; x < 1.0; x += 0.5) {
        pc.points.emplace_back(x,1.0f,1.0f);
        pc.colors.emplace_back(1.0f,1.0f,1.0f);
        /*for(float y = -1.0f; y < 1.0; y += 0.5) {
            for(float z = -1.0f; z < 1.0; z += 0.5) {
                pc.points.emplace_back(x,y,z);
                pc.colors.emplace_back((x+1)/2.0f,(y+1)/2.0f,(z+1)/2.0f);
            }
        }*/
    }

    std::cout << "point_cloud:" << std::endl;
    std::cout << " > size:" << pc.size() << "\n";
    /*for(auto p : pc.points)
        std::cout << "  > " << p.x << "," << p.y << "," << p.z << std::endl;*/
    //// ENCODING

    PointCloudGridEncoder encoder;
    auto start = std::chrono::steady_clock::now();
    zmq::message_t msg = encoder.encode<uint8_t, uint16_t>(&pc, Vec8(2,2,2));
    auto end = std::chrono::steady_clock::now();

    unsigned elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    //std::cout << "Encoding took " << elapsed << "ms.\n";
    int size_bytes = msg.size();
    int size_bit = size_bytes * 8;
    float mbit = size_bit / 1000000.0f;
    /*
    std::cout << "Message Size\n"
              << "  > bytes " << size_bytes << "\n"
              << "  > mbit " << mbit << "\n";
    */
    //// DECODING
    PointCloud<Vec<float>, Vec<float>> pc2;
    start = std::chrono::steady_clock::now();
    bool success = encoder.decode(msg, &pc2);
    end = std::chrono::steady_clock::now();

    std::cout << "point_cloud2:" << std::endl;
    std::cout << " > size:" << pc2.size() << "\n";
    /*for(auto p : pc2.points)
        std::cout << "  > " << p.x << "," << p.y << "," << p.z << std::endl;*/

    elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    std::cout << "Decoding took " << elapsed << "ms.\n";
    if(success)
        std::cout << "  > success: YES\n";
    else
        std::cout << "  > success: NO\n";


    Measure t;
    t.startWatch();
    std::cout << "MSE " << t.meanSquaredErrorPC(pc, pc2) << std::endl;
    t.printTimeSpan(t.stopWatch());

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
