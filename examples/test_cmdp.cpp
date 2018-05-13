#include <zmq.hpp>

#include "../include/Measure.hpp"
#include "../include/CMDParser.hpp"
#include "../include/PointCloudGridEncoder.hpp"
#include "../include/BinaryFile.hpp"

/*
 * Sample usage for libpcc.
 *  - extracts an uncompressed point cloud from a file in binary format.
 *  - sets up compression settings
 *  - encodes uncompressed point cloud
 *  - appends/reads sample data to message using message appendix
 *  - decodes compressed message
 *  - prints statistics
 *  - shows sample usage for sending compressed data over tcp socket using zmq
 *    - (uncomment block comments to try)
*/
int main(int argc, char* argv[]){
    /* // initialize zmq tcp socket
    CMDParser p("socket");
    p.init(argc,argv);

    std::string socket_name(p.getArgs()[0]);

    zmq::context_t ctx(1); // means single threaded
    zmq::socket_t socket(ctx, ZMQ_PUB); // means a publisher

    uint32_t hwm = 1;
    socket.setsockopt(ZMQ_SNDHWM, &hwm, sizeof(hwm));

    std::string endpoint("tcp://" + socket_name);
    socket.bind(endpoint.c_str());
    */

    // Define EncoderSettings
    PointCloudGridEncoder encoder;
    BoundingBox bb(Vec<float>(-1.0f, 0.0f, -1.0f), Vec<float>(1.0f, 2.2f, 1.0f));
    encoder.settings.grid_precision = GridPrecisionDescriptor(
        Vec8(6,6,6),
        bb,
        Vec<BitCount>(BIT_6,BIT_6,BIT_6),
        Vec<BitCount>(BIT_4,BIT_4,BIT_4)
    );
    encoder.settings.irrelevance_coding = true;
    encoder.settings.entropy_coding = true;
    encoder.settings.appendix_size = 500;
    encoder.settings.verbose = true;

    // Read uncompressed point cloud data from file
    std::vector<UncompressedVoxel> pc;
    BinaryFile file;
    std::cout << "================= FILE INFO ==================\n";
    if(file.read("./voxel_log.txt")) {
        pc.resize(file.getSize() / sizeof(UncompressedVoxel));
        file.copy((char*) pc.data());
        std::cout << "SUCCESS\n";
    }
    else {
        std::cout << "FAILURE\n";
    }
    std::cout << "==============================================\n";

    std::cout << "\n============== COMPRESSION INFO ==============\n";
    zmq::message_t msg = encoder.encode(pc);
    encoder.writeToAppendix(msg, "SOME USEFUL INFORMATION");

    float size_uncomp = (file.getSize() * 8) / 1000000.0f;
    float size_comp = msg.size() / 1000000.0f;
    std::cout << "OVERALL STATS\n";
    std::cout << "  > Size uncompressed (Mbits): " << size_uncomp << std::endl;
    std::cout << "  > Size compressed (Mbits): " << size_comp << std::endl;
    std::cout << "  > Size uncompressed @ 30fps (Mbits): " << size_uncomp*30.0f << std::endl;
    std::cout << "  > Size compressed @ 30fps (Mbits): " << size_comp*30.0f << std::endl;
    std::cout << "  > Remaining size: ~" << (int) ((size_comp / size_uncomp) * 100.0f) << "%\n";
    std::cout << "==============================================\n";

    /*
    // send compressed point cloud
    while(true) {
        socket.send(msg);
    }
    */

    std::cout << "\n============= DECOMPRESSION INFO =============\n";
    std::vector<UncompressedVoxel> pc_comp;
    if(encoder.decode(msg, &pc_comp)) {
        std::cout << "SUCCCESS\n";
        std::cout << "  > Points extracted: " << pc_comp.size() << std::endl;
        // extract Appendix
        std::string appendix;
        encoder.readFromAppendix(msg, appendix);
        std::cout << "  > Appendix data: " << appendix << std::endl;
    }
    else {
        std::cout << "FAILURE\n";
    }
    std::cout << "==============================================\n";

    return 0;
}
