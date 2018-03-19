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

    //________________________________________________________________________
    /*std::vector<std::string> files;
    files.push_back("./snap-8-6-4de.txt");
    files.push_back("./snap-8-7-6de.txt");
    files.push_back("./snap-16-8-8de.txt");
    files.push_back("./snap-16-16-8de.txt");
    files.push_back("./snap-16-32-8de.txt");
    files.push_back("./snap-8-8-8de.txt");
    files.push_back("./snap-4-8-8de.txt");
    files.push_back("./snap-16-7-8de.txt");
    files.push_back("./snap-8-10-8de.txt");
    files.push_back("./snap-4-12-8de.txt");
    files.push_back("./snap-8-8-7de.txt");
    files.push_back("./snap-8-8-6de.txt");
    files.push_back("./snap-8-8-5de.txt");
    files.push_back("./snap-8-8-4de.txt");
    files.push_back("./snap-8-8-3de.txt");
    files.push_back("./snap-16-8-7de.txt");
    files.push_back("./snap-16-8-6de.txt");
    files.push_back("./snap-16-8-5de.txt");
    files.push_back("./snap-16-8-4de.txt");
    files.push_back("./snap-16-8-3de.txt");
    files.push_back("./snap-8-16-8de.txt");
    files.push_back("./snap-8-32-8de.txt");
*/
    PointCloudGridEncoder encoder;
    BoundingBox bb(Vec<float>(-1.0f, 0.0f, -1.0f), Vec<float>(1.0f, 2.2f, 1.0f));
    encoder.settings.grid_precision = GridPrecisionDescriptor(
            Vec8(8,8,8),
            bb,
            Vec<BitCount>(BIT_8,BIT_8,BIT_8),
            Vec<BitCount>(BIT_8,BIT_8,BIT_8)
    );
/*
    std::vector<UncompressedVoxel> v_raw;
    BinaryFile raw;
    if(raw.read("./clean.txt")) {
        v_raw.resize(raw.getSize() / sizeof(UncompressedVoxel));
        raw.copy((char*) v_raw.data());
        std::cout << "READ raw voxels from file done.\n";
    }
    else {
        std::cout << "READ raw voxels from file failed.\n";
    }

    std::cout << "Encoding Type, Position Error Avg, Position Error Max, Color Error Avg, Color Error Max" << std::endl;
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

        v_raw[0].pos; // float[3]
        v_raw[0].color_rgba; // unsigned char[4] aaaaaaaargb
        PointCloud<Vec<float>, Vec<float>> pc;
        for(auto voxel : v_raw) {
            pc.points.push_back(Vec<float>(voxel.pos[0],voxel.pos[1],voxel.pos[2]));
            pc.colors.push_back(Vec<float>(((float)voxel.color_rgba[1]) / 255.0f, ((float) voxel.color_rgba[2]) / 255.0f, ((float) voxel.color_rgba[3]) / 255.0f));
            //std::cout << voxel.pos[0] << ", " << voxel.pos[1] << ", " << voxel.pos[2] << std::endl;
            //std::cout << ((int)voxel.color_rgba[1]) << ", " << ((int)voxel.color_rgba[2]) << ", " << ((int)voxel.color_rgba[1])<< std::endl;
        }
        v_dec[0].pos; // float[3]
        v_dec[0].color_rgba; // unsigned char[4] aaaaaaaargb
        PointCloud<Vec<float>, Vec<float>> pc2;
        for(auto voxel : v_dec) {
            pc2.points.push_back(Vec<float>(voxel.pos[0],voxel.pos[1],voxel.pos[2]));
            pc2.colors.push_back(Vec<float>(((float)voxel.color_rgba[1]) / 255.0f, ((float) voxel.color_rgba[2]) / 255.0f, ((float) voxel.color_rgba[3]) / 255.0f));
        }

        Measure m;
        std::vector<float> results = m.comparePC(pc, pc2, bb);

        std::cout << indx << ", ";
        std::cout << results[0] << ", ";
        std::cout << results[2] << ", ";
        std::cout << results[3] << ", ";
        std::cout << results[5] << std::endl;
    }*/

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
    std::cout << "RAW VOXEL data (parsed from file) \n";
    std::cout << "  > voxel count " << v_raw_pic.size() << std::endl;

    PointCloud<Vec<float>, Vec<float>> pc_raw_pic(BoundingBox(Vec<float>(-1.01f,-1.01f,-1.01f), Vec<float>(1.01f,1.01f,1.01f)));
int count = 0;
    for(auto index : v_raw_pic) {
      pc_raw_pic.points.emplace_back(index.pos[0], index.pos[1], index.pos[2]);
      pc_raw_pic.colors.emplace_back(((float) index.color_rgba[0]) / 255.0f, ((float) index.color_rgba[0]) / 255.0f, ((float) index.color_rgba[0]) / 255.0f);

      if(count < 10) {
        std::cout << "PC Color check: " <<(float) index.color_rgba[0] << ", " <<(float) index.color_rgba[1] << ", " <<(float) index.color_rgba[2] << ", " << index.color_rgba[3] << std::endl;
        std::cout << "PC Pos check: " << index.pos[0] << ", " << index.pos[1] << ", " << index.pos[2] << std::endl;
        count++;
      }
    }

    zmq::message_t msg_raw_pic = encoder.encode(v_raw_pic);
    std::vector<UncompressedVoxel> msg_decoded_pic;
    encoder.decode(msg_raw_pic, & msg_decoded_pic);

    PointCloud<Vec<float>, Vec<float>> pc_pcc(BoundingBox(Vec<float>(-1.01f,-1.01f,-1.01f), Vec<float>(1.01f,1.01f,1.01f)));

    for(auto index : msg_decoded_pic) {
      pc_pcc.points.emplace_back(index.pos[0], index.pos[1], index.pos[2]);
      pc_pcc.colors.emplace_back(((float) index.color_rgba[0]) / 255.0f, ((float) index.color_rgba[0]) / 255.0f, ((float) index.color_rgba[0]) / 255.0f);
    }

    std::vector<UncompressedVoxel> v_comp_pic;
    std::cout << "Read raw Data from Picture based approach" << std::endl;
    BinaryFile comp_pic;
    if(comp_pic.read("./comp_nint.txt")) {
        v_comp_pic.resize(comp_pic.getSize() / sizeof(UncompressedVoxel));
        comp_pic.copy((char*) v_comp_pic.data());
        std::cout << "READ raw voxels from file done.\n";
    }
    else {
        std::cout << "READ raw voxels from file failed.\n";
    }
    std::cout << "RAW VOXEL data (parsed from file) \n";
    std::cout << "  > voxel count " << v_comp_pic.size() << std::endl;

    PointCloud<Vec<float>, Vec<float>> pc_comp_pic(BoundingBox(Vec<float>(-1.01f,-1.01f,-1.01f), Vec<float>(1.01f,1.01f,1.01f)));

    for(auto index : v_comp_pic) {
      pc_comp_pic.points.emplace_back(index.pos[0], index.pos[1], index.pos[2]);
      pc_comp_pic.colors.emplace_back(((float) index.color_rgba[0]) / 255.0f, ((float) index.color_rgba[1]) / 255.0f, ((float) index.color_rgba[2]) / 255.0f);
      // std::cout << "PC Color check: " <<(float) index.color_rgba[0] << ", " <<(float) index.color_rgba[1] << ", " <<(float) index.color_rgba[2] << ", " << index.color_rgba[3] << std::endl;
      // std::cout << "PC Pos check: " << index.pos[0] << ", " << index.pos[1] << ", " << index.pos[2] << std::endl;
    }

    std::vector<UncompressedVoxel> v_comp_int_pic;
    std::cout << "Read raw Data from Picture based approach" << std::endl;
    BinaryFile comp_int_pic;
    if(comp_int_pic.read("./comp_int.txt")) {
        v_comp_int_pic.resize(comp_int_pic.getSize() / sizeof(UncompressedVoxel));
        comp_int_pic.copy((char*) v_comp_int_pic.data());
        std::cout << "READ raw voxels from file done.\n";
    }
    else {
        std::cout << "READ raw voxels from file failed.\n";
    }
    std::cout << "RAW VOXEL data (parsed from file) \n";
    std::cout << "  > voxel count " << v_comp_int_pic.size() << std::endl;

    PointCloud<Vec<float>, Vec<float>> pc_comp_int_pic(BoundingBox(Vec<float>(-1.01f,-1.01f,-1.01f), Vec<float>(1.01f,1.01f,1.01f)));
count = 0;
    for(auto index : v_comp_int_pic) {
      pc_comp_pic.points.emplace_back(index.pos[0], index.pos[1], index.pos[2]);
      pc_comp_pic.colors.emplace_back(((float) index.color_rgba[0]) / 255.0f,((float) index.color_rgba[0]) / 255.0f, ((float) index.color_rgba[0]) / 255.0f);
      if(count < 10) {
        std::cout << "PC Color check: " <<(float) index.color_rgba[0] << ", " <<(float) index.color_rgba[1] << ", " <<(float) index.color_rgba[2] << ", " << index.color_rgba[3] << std::endl;
        std::cout << "PC Pos check: " << index.pos[0] << ", " << index.pos[1] << ", " << index.pos[2] << std::endl;
        count++;
      }
    }

    std::cout << "comp size (without interframe): " << v_comp_pic.size() << " / " << v_comp_pic.size() / sizeof(UncompressedVoxel) << std::endl;
    std::cout << "comp size (with interframe): " << v_comp_int_pic.size() << " / " << v_comp_int_pic.size() / sizeof(UncompressedVoxel) << std::endl;
    std::cout << "comp size (with pcc): " << msg_decoded_pic.size() << " / " << msg_decoded_pic.size() / sizeof(UncompressedVoxel) << std::endl;
    std::cout << "comp raw size: " << v_raw_pic.size() << " / " << v_raw_pic.size() / sizeof(UncompressedVoxel) << std::endl;

    Measure t;
    std::vector<float> pic_nint = t.comparePC(pc_raw_pic, pc_comp_pic, bb);
    std::vector<float> pic_int = t.comparePC(pc_raw_pic, pc_comp_int_pic, bb);
    std::vector<float> pic_pc = t.comparePC(pc_raw_pic, pc_pcc, bb);
    std::vector<float> pc_pic = t.comparePC(pc_pcc, pc_raw_pic, bb);



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


    //m.printResultsPC(results);

/*
    std::vector<UncompressedVoxel> v_decomp;
    BinaryFile decomp;
    if(decomp.read("./snap-8-8-8de.txt")) {
        v_decomp.resize(decomp.getSize() / sizeof(UncompressedVoxel));
        decomp.copy((char*) v_decomp.data());
        std::cout << "READ decompressed voxels from file done.\n";
    }
    else {
        std::cout << "READ decompressed voxels from file failed.\n";
    }

    v_decomp[0].pos; // float[3]
    v_decomp[0].color_rgba; // unsigned char[4] aaaaaaaargb
    PointCloud<Vec<float>, Vec<float>> pc2;
    for(auto voxel : v_decomp) {
        pc2.points.push_back(Vec<float>(voxel.pos[0],voxel.pos[1],voxel.pos[2]));
        pc2.colors.push_back(Vec<float>(((float)voxel.color_rgba[1]) / 255.0f, ((float) voxel.color_rgba[2]) / 255.0f, ((float) voxel.color_rgba[3]) / 255.0f));
    }

/*
    std::vector<UncompressedVoxel> v_raw;
    BinaryFile raw;
    if(raw.read("./raw_data.txt")) {
        v_raw.resize(raw.getSize() / sizeof(UncompressedVoxel));
        raw.copy((char*) v_raw.data());
        std::cout << "READ raw voxels from file done.\n";
    }
    else {
        std::cout << "READ raw voxels from file failed.\n";
    }
    std::cout << "RAW VOXEL data (parsed from file) \n";
    std::cout << "  > voxel count " << v_raw.size() << std::endl;

    v_raw[0].pos; // float[3]
    v_raw[0].color_rgba; // unsigned char[4] aaaaaaaargb
    PointCloud<Vec<float>, Vec<float>> pc;
    for(auto voxel : v_raw) {
        pc.points.push_back(Vec<float>(voxel.pos[0],voxel.pos[1],voxel.pos[2]));
        pc.colors.push_back(Vec<float>(((float)voxel.color_rgba[1]) / 255.0f, ((float) voxel.color_rgba[2]) / 255.0f, ((float) voxel.color_rgba[3]) / 255.0f));

        //std::cout << ((float)voxel.color_rgba[1] / 255.0f) << ", " << ((float)voxel.color_rgba[2] / 255.0f) << ", " << ((float)voxel.color_rgba[3] / 255.0f) << std::endl;
    }

    std::vector<UncompressedVoxel> v_decomp;
    BinaryFile decomp;
    if(decomp.read("./decomp_data.txt")) {
        v_decomp.resize(decomp.getSize() / sizeof(UncompressedVoxel));
        decomp.copy((char*) v_decomp.data());
        std::cout << "READ raw voxels from file done.\n";
    }
    else {
        std::cout << "READ raw voxels from file failed.\n";
    }
    std::cout << "RAW VOXEL data (parsed from file) \n";
    std::cout << "  > voxel count " << v_decomp.size() << std::endl;

    v_decomp[0].pos; // float[3]
    v_decomp[0].color_rgba; // unsigned char[4] aaaaaaaargb
    PointCloud<Vec<float>, Vec<float>> pc2;
    for(auto voxel : v_decomp) {
        pc2.points.push_back(Vec<float>(voxel.pos[0],voxel.pos[1],voxel.pos[2]));
        pc2.colors.push_back(Vec<float>(((float)voxel.color_rgba[1]) / 255.0f, ((float) voxel.color_rgba[2]) / 255.0f, ((float) voxel.color_rgba[3]) / 255.0f));
    }
*/



    /*
    // ENCODER SETUP
    PointCloudGridEncoder encoder;
    // settings should match './grid_log_info.txt'
    BoundingBox bb(Vec<float>(-1.0f,0.05f,-1.0f), Vec<float>(1.0f,2.2f,1.0f));
    encoder.settings.grid_precision = GridPrecisionDescriptor(
            Vec8(8,8,8), // dimensions
            bb,
            Vec<BitCount>(BIT_4,BIT_4,BIT_4), // default point encoding
            Vec<BitCount>(BIT_8,BIT_8,BIT_8)  // default color encoding
    );
    encoder.settings.verbose = true; //extended information

    // READ RAW DATA FROM FILE
    std::vector<UncompressedVoxel> v_raw;
    BinaryFile raw;
    if(raw.read("./voxel_log.txt")) {
        v_raw.resize(raw.getSize() / sizeof(UncompressedVoxel));
        raw.copy((char*) v_raw.data());
        std::cout << "READ raw voxels from file done.\n";
    }
    else {
        std::cout << "READ raw voxels from file failed.\n";
    }
    std::cout << "RAW VOXEL data (parsed from file) \n";
    std::cout << "  > voxel count " << v_raw.size() << std::endl;

    float avg_clr[4] = {0.0f,0.0f,0.0f,0.0f};
    float avg_pos[3] = {0.0f,0.0f,0.0f};
    int skipped = 0;
    for(auto voxel:v_raw) {
        avg_clr[0] += (static_cast<float>(voxel.color_rgba[0]));
        avg_clr[1] += (static_cast<float>(voxel.color_rgba[1]));
        avg_clr[2] += (static_cast<float>(voxel.color_rgba[2]));
        avg_clr[3] += (static_cast<float>(voxel.color_rgba[3]));
        if(bb.contains(voxel.pos)) {
            avg_pos[0] += voxel.pos[0];
            avg_pos[1] += voxel.pos[1];
            avg_pos[2] += voxel.pos[2];
        }
        else {
            ++skipped;
        }
    }
    avg_clr[0] /= v_raw.size();
    avg_clr[1] /= v_raw.size();
    avg_clr[2] /= v_raw.size();
    avg_clr[3] /= v_raw.size();
    avg_pos[0] /= (v_raw.size() - skipped);
    avg_pos[1] /= (v_raw.size() - skipped);
    avg_pos[2] /= (v_raw.size() - skipped);

    std::cout << "  > avg color "
              << avg_clr[0] << ","
              << avg_clr[1] << ","
              << avg_clr[2] << ","
              << avg_clr[3] << std::endl;
    std::cout << "  > avg pos "
              << avg_pos[0] << ","
              << avg_pos[1] << ","
              << avg_pos[2] << std::endl;

    zmq::message_t msg_v_raw = encoder.encode(v_raw);
    std::vector<UncompressedVoxel> msg_v_raw_decoded;
    encoder.decode(msg_v_raw, &msg_v_raw_decoded);
    std::cout << "DECODED message (encoded using raw voxels)\n";
    std::cout << "  > voxel count " << msg_v_raw_decoded.size() << std::endl;

    avg_clr[0] = 0.0f;
    avg_clr[1] = 0.0f;
    avg_clr[2] = 0.0f;
    avg_clr[3] = 0.0f;
    avg_pos[0] = 0.0f;
    avg_pos[1] = 0.0f;
    avg_pos[2] = 0.0f;
    for(auto voxel:msg_v_raw_decoded) {
        avg_clr[0] += (static_cast<float>(voxel.color_rgba[0]));
        avg_clr[1] += (static_cast<float>(voxel.color_rgba[1]));
        avg_clr[2] += (static_cast<float>(voxel.color_rgba[2]));
        avg_clr[3] += (static_cast<float>(voxel.color_rgba[3]));
        avg_pos[0] += voxel.pos[0];
        avg_pos[1] += voxel.pos[1];
        avg_pos[2] += voxel.pos[2];
    }
    avg_clr[0] /= msg_v_raw_decoded.size();
    avg_clr[1] /= msg_v_raw_decoded.size();
    avg_clr[2] /= msg_v_raw_decoded.size();
    avg_clr[3] /= msg_v_raw_decoded.size();
    avg_pos[0] /= msg_v_raw_decoded.size();
    avg_pos[1] /= msg_v_raw_decoded.size();
    avg_pos[2] /= msg_v_raw_decoded.size();

    std::cout << "  > avg color "
              << avg_clr[0] << ","
              << avg_clr[1] << ","
              << avg_clr[2] << ","
              << avg_clr[3] << std::endl;
    std::cout << "  > avg pos "
              << avg_pos[0] << ","
              << avg_pos[1] << ","
              << avg_pos[2] << std::endl;
    */
    /*
    Measure t;

    PointCloud<Vec<float>, Vec<float>> pc(BoundingBox(Vec<float>(-1.01f,-1.01f,-1.01f), Vec<float>(1.01f,1.01f,1.01f)));
    std::vector<UncompressedVoxel> pc_vec;
    for(float x = -1.0f; x < 1.0; x += 0.04) {
        for(float y = -1.0f; y < 1.0; y += 0.04) {
            for(float z = -1.0f; z < 1.0; z += 0.04) {
                pc.points.emplace_back(x,y,z);
                pc.colors.emplace_back((x+1)/2.0f,(y+1)/2.0f,(z+1)/2.0f);
                pc_vec.push_back(UncompressedVoxel());
                pc_vec.back().pos[0] = x;
                pc_vec.back().pos[1] = y;
                pc_vec.back().pos[2] = z;
                pc_vec.back().color_rgba[0] = static_cast<unsigned char>(pc.colors.back().x*255.0f);
                pc_vec.back().color_rgba[1] = static_cast<unsigned char>(pc.colors.back().y*255.0f);
                pc_vec.back().color_rgba[2] = static_cast<unsigned char>(pc.colors.back().z*255.0f);
                pc_vec.back().color_rgba[3] = 255;
            }
        }
    }
    PointCloud<Vec<float>, Vec<float>> pc3(BoundingBox(Vec<float>(-1.01f,-1.01f,-1.01f), Vec<float>(1.01f,1.01f,1.01f)));
    for(float x = -1.0f; x < 1.0; x += 0.5) {
        for(float y = -1.0f; y < 1.0; y += 0.5) {
            for(float z = -1.0f; z < 1.0; z += 0.5) {
                pc3.points.emplace_back(x,y,z);
                pc3.colors.emplace_back((x+1)/2.0f,(y+1)/2.0f,(z+1)/2.0f);
            }
        }
    }
    PointCloud<Vec<float>, Vec<float>> pc4(BoundingBox(Vec<float>(-1.01f,-1.01f,-1.01f), Vec<float>(1.01f,1.01f,1.01f)));
    pc4.points.emplace_back(1.0f,1.0f,1.0f);
    pc4.points.emplace_back(1.0f,1.0f,0.0f);
    pc4.points.emplace_back(1.0f,0.0f,0.0f);
    pc4.colors.emplace_back(0.5f,0.5f,0.5f);
    pc4.colors.emplace_back(0.5f,0.5f,0.5f);
    pc4.colors.emplace_back(0.5f,0.5f,0.5f);

    PointCloud<Vec<float>, Vec<float>> pc5(BoundingBox(Vec<float>(-1.01f,-1.01f,-1.01f), Vec<float>(1.01f,1.01f,1.01f)));
    pc5.points.emplace_back(1.0f,1.0f,1.2f);
    pc5.points.emplace_back(1.0f,1.0f,0.4f);
    pc5.points.emplace_back(0.2f,0.0f,0.0f);
    pc5.colors.emplace_back(0.5f,0.5f,0.5f);
    pc5.colors.emplace_back(0.5f,0.5f,0.5f);
    pc5.colors.emplace_back(0.45f,0.5f,0.5f);



    std::cout << "POINT CLOUD" << std::endl;
    std::cout << "  > size " << pc.size() << "\n";

    //// ENCODING

    PointCloudGridEncoder encoder;
    encoder.settings.grid_precision = GridPrecisionDescriptor(
            Vec8(4,4,4), // dimensions
            pc.bounding_box, // bounding box
            Vec<BitCount>(BIT_4,BIT_4,BIT_4), // default point encoding
            Vec<BitCount>(BIT_8,BIT_8,BIT_8)  // default color encoding
    );
    encoder.settings.num_threads = 24;

    t.startWatch();
    zmq::message_t msg = encoder.encode(pc_vec);

    std::cout << "ENCODING DONE in " << t.stopWatch() << "ms.\n";
    auto size_bytes = static_cast<int>(msg.size());
    int size_bit = size_bytes * 8;
    float mbit = size_bit / 1000000.0f;

    std::cout << "  > Message Size\n"
              << "    > bytes " << size_bytes << "\n"
              << "    > mbit " << mbit << "\n";
    */

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
    /*
    PointCloud<Vec<float>, Vec<float>> pc2;
    std::vector<UncompressedVoxel> pc_vec2;
    t.startWatch();
    //zmq::message_t msg2 = f.get();
    bool success = encoder.decode(msg, &pc_vec2);
    std::cout << "DECODING DONE in " << t.stopWatch() << "ms.\n";
    std::cout << "  > size " << pc_vec2.size() << "\n";
    if(success)
        std::cout << "  > success: YES\n";
    else
        std::cout << "  > success: NO\n";
    */

    /*
    t.startWatch();
    std::vector<float> results = t.comparePC(pc4, pc5);
    t.printResultsPC(results);

    // std::cout << "  > MSE " << results[0] << std::endl;
    // std::cout << "  > CLR ERROR " << results[1] << std::endl;
    // std::cout << "    > took " << t.stopWatch() << "ms" << std::endl;

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
