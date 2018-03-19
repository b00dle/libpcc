#include "../include/Measurement.hpp"
#include "../include/CMDParser.hpp"

#include <zmq.hpp>
#include <zlib.h>


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
    socket.setsockopt(ZMQ_SNDHWM, &hwm, sizeof(hwm));

    std::string endpoint("tcp://" + socket_name);
    socket.bind(endpoint.c_str());
    */

    //Definie EncoderSettings
    PointCloudGridEncoder encoder;
    BoundingBox bb(Vec<float>(-1.0f, 0.0f, -1.0f), Vec<float>(1.0f, 2.2f, 1.0f));
    encoder.settings.grid_precision = GridPrecisionDescriptor(
      Vec8(8,8,8),
      bb,
      Vec<BitCount>(BIT_8,BIT_8,BIT_8),
      Vec<BitCount>(BIT_8,BIT_8,BIT_8)
    );
    encoder.settings.irrelevance_coding = true;
    encoder.settings.entropy_coding = true;

    // read raw reference file
    std::vector<UncompressedVoxel> v_raw_pic;
    std::cout << "Read raw Data from Picture based approach" << std::endl;
    BinaryFile raw_pic;
    if(raw_pic.read("./raw_data.txt")) {
      v_raw_pic.resize(raw_pic.getSize() / sizeof(UncompressedVoxel));
      raw_pic.copy((char*) v_raw_pic.data());
      std::cout << "READ raw voxels from file done.\n";
    }
    else {
      std::cout << "READ raw voxels from file failed.\n";
    }

    // write to pc
    PointCloud<Vec<float>, Vec<float>> pc_raw_pic(BoundingBox(Vec<float>(-1.01f,-1.01f,-1.01f), Vec<float>(1.01f,1.01f,1.01f)));
    for(auto index : v_raw_pic) {
      pc_raw_pic.points.emplace_back(index.pos[0], index.pos[1], index.pos[2]);
      pc_raw_pic.colors.emplace_back(((float) index.color_rgba[0]) / 255.0f, ((float) index.color_rgba[0]) / 255.0f, ((float) index.color_rgba[0]) / 255.0f);
    }
    /*
    // write multiple files to vector to access later
    std::vector<std::string> files;
    files.push_back("./snap-8-6-4de.txt");
    files.push_back("./snap-8-7-6de.txt");
    files.push_back("./snap-16-8-8de.txt");

    // access files
    for(auto indx : files) {
        std::vector<UncompressedVoxel> v_dec;
        BinaryFile dec;
        if(dec.read(indx)) {
            v_dec.resize(dec.getSize() / sizeof(UncompressedVoxel));
            dec.copy((char*) v_dec.data());
        //    std::cout << "READ dec voxels from file done.\n";
        }
        else {
            std::cout << "READ dec voxels from file failed.\n";
        }
        // fill pointcloud
        v_dec[0].pos; // float[3]
        v_dec[0].color_rgba; // unsigned char[4] aaaaaaaargb
        PointCloud<Vec<float>, Vec<float>> pc;
        for(auto voxel : v_dec) {
            pc.points.push_back(Vec<float>(voxel.pos[0],voxel.pos[1],voxel.pos[2]));
            pc.colors.push_back(Vec<float>(((float)voxel.color_rgba[1]) / 255.0f, ((float) voxel.color_rgba[2]) / 255.0f, ((float) voxel.color_rgba[3]) / 255.0f));
        }

        // compare whole input array to raw / base pointcloud
        Measure m;
        std::vector<float> results = m.comparePC(pc_raw_pic, pc, bb);

        // print results
        std::cout << indx << ", ";
        std::cout << results[0] << ", ";
        std::cout << results[2] << ", ";
        std::cout << results[3] << ", ";
        std::cout << results[5] << std::endl;
    }*/

    // write pc to zmq msg
    // encode & decode msg
    // write decoded msg to voxelvec
    zmq::message_t msg_raw_pic = encoder.encode(v_raw_pic);
    std::vector<UncompressedVoxel> msg_decoded_pic;
    encoder.decode(msg_raw_pic, & msg_decoded_pic);

    // create uncompressed pc
    PointCloud<Vec<float>, Vec<float>> pc_pcc(BoundingBox(Vec<float>(-1.01f,-1.01f,-1.01f), Vec<float>(1.01f,1.01f,1.01f)));
    for(auto index : msg_decoded_pic) {
      pc_pcc.points.emplace_back(index.pos[0], index.pos[1], index.pos[2]);
      pc_pcc.colors.emplace_back(((float) index.color_rgba[0]) / 255.0f, ((float) index.color_rgba[0]) / 255.0f, ((float) index.color_rgba[0]) / 255.0f);
    }



    // current tests pipeline comparison
    std::vector<UncompressedVoxel> v_comp_pic;
    std::cout << "Read 'compressed' from Picture based approach" << std::endl;
    BinaryFile comp_pic;
    if(comp_pic.read("./comp_nint.txt")) {
        v_comp_pic.resize(comp_pic.getSize() / sizeof(UncompressedVoxel));
        comp_pic.copy((char*) v_comp_pic.data());
        std::cout << "READ raw voxels from file done.\n";
    }
    else {
        std::cout << "READ raw voxels from file failed.\n";
    }
    PointCloud<Vec<float>, Vec<float>> pc_comp_pic(BoundingBox(Vec<float>(-1.01f,-1.01f,-1.01f), Vec<float>(1.01f,1.01f,1.01f)));
    for(auto index : v_comp_pic) {
      pc_comp_pic.points.emplace_back(index.pos[0], index.pos[1], index.pos[2]);
      pc_comp_pic.colors.emplace_back(((float) index.color_rgba[0]) / 255.0f, ((float) index.color_rgba[1]) / 255.0f, ((float) index.color_rgba[2]) / 255.0f);
      // std::cout << "PC Pos check: " << index.pos[0] << ", " << index.pos[1] << ", " << index.pos[2] << std::endl;
      // std::cout << "PC Color check: " <<(float) index.color_rgba[0] << ", " <<(float) index.color_rgba[1] << ", " <<(float) index.color_rgba[2] << ", " << index.color_rgba[3] << std::endl;
    }

    std::vector<UncompressedVoxel> v_comp_int_pic;
    std::cout << "Read 'compressed interframe' from Picture based approach" << std::endl;
    BinaryFile comp_int_pic;
    if(comp_int_pic.read("./comp_int.txt")) {
        v_comp_int_pic.resize(comp_int_pic.getSize() / sizeof(UncompressedVoxel));
        comp_int_pic.copy((char*) v_comp_int_pic.data());
        std::cout << "READ raw voxels from file done.\n";
    }
    else {
        std::cout << "READ raw voxels from file failed.\n";
    }
    PointCloud<Vec<float>, Vec<float>> pc_comp_int_pic(BoundingBox(Vec<float>(-1.01f,-1.01f,-1.01f), Vec<float>(1.01f,1.01f,1.01f)));
    for(auto index : v_comp_int_pic) {
      pc_comp_pic.points.emplace_back(index.pos[0], index.pos[1], index.pos[2]);
      pc_comp_pic.colors.emplace_back(((float) index.color_rgba[0]) / 255.0f,((float) index.color_rgba[0]) / 255.0f, ((float) index.color_rgba[0]) / 255.0f);
    }

    // print num of points
    std::cout << "comp size / num points (without interframe): " << v_comp_pic.size() << " / " << v_comp_pic.size() / sizeof(UncompressedVoxel) << std::endl;
    std::cout << "comp size / num points (with interframe): " << v_comp_int_pic.size() << " / " << v_comp_int_pic.size() / sizeof(UncompressedVoxel) << std::endl;
    std::cout << "comp size / num points (with pcc): " << msg_decoded_pic.size() << " / " << msg_decoded_pic.size() / sizeof(UncompressedVoxel) << std::endl;
    std::cout << "comp raw size / num points: " << v_raw_pic.size() << " / " << v_raw_pic.size() / sizeof(UncompressedVoxel) << std::endl;

    Measure t;
    // compare different approaches
    std::vector<float> pic_nint = t.comparePC(pc_raw_pic, pc_comp_pic, bb);
    std::vector<float> pic_int = t.comparePC(pc_raw_pic, pc_comp_int_pic, bb);
    std::vector<float> pic_pc = t.comparePC(pc_raw_pic, pc_pcc, bb);
    std::vector<float> pc_pic = t.comparePC(pc_pcc, pc_raw_pic, bb);

    // print results
    std::cout << "avg Error" << ", ";
    std::cout << "max Error" << ", ";
    std::cout << "avg ClrError" << ", ";
    std::cout << "max ClrError" << std::endl;
    std::cout << "Results for Pic with itself without interframe" << std::endl;
    std::cout << pic_nint[0] << ", ";
    std::cout << pic_nint[2] << ", ";
    std::cout << pic_nint[3] << ", ";
    std::cout << pic_nint[5] << std::endl;
    std::cout << std::endl;

    std::cout << "Results for Pic with itself with interframe" << std::endl;
    std::cout << pic_int[0] << ", ";
    std::cout << pic_int[2] << ", ";
    std::cout << pic_int[3] << ", ";
    std::cout << pic_int[5] << std::endl;
    std::cout << std::endl;

    std::cout << "Results for Pic with PC" << std::endl;
    std::cout << pic_pc[0] << ", ";
    std::cout << pic_pc[2] << ", ";
    std::cout << pic_pc[3] << ", ";
    std::cout << pic_pc[5] << std::endl;
    std::cout << pc_pic[0] << ", ";
    std::cout << pc_pic[2] << ", ";
    std::cout << pc_pic[3] << ", ";
    std::cout << pc_pic[5] << std::endl;

    return 0;
}
