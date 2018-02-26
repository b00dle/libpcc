#include "../include/Measurement.hpp"
#include "../include/CMDParser.hpp"

#include <zmq.hpp>
#include <zlib.h>


#include "../include/PointCloudGridEncoder.hpp"
#include "../include/BinaryFile.hpp"



unsigned long file_size(char *filename)
{
    FILE *pFile = fopen(filename, "rb");
    fseek (pFile, 0, SEEK_END);
    unsigned long size = ftell(pFile);
    fclose (pFile);
    return size;
}

int decompress_one_file(char *infilename, char *outfilename)
{
    gzFile infile = gzopen(infilename, "rb");
    FILE *outfile = fopen(outfilename, "wb");
    if (!infile || !outfile) return -1;

    char buffer[128];
    int num_read = 0;
    while ((num_read = gzread(infile, buffer, sizeof(buffer))) > 0) {
        fwrite(buffer, 1, num_read, outfile);
    }

    gzclose(infile);
    fclose(outfile);
}

int compress_one_file(char *infilename, char *outfilename)
{
    FILE *infile = fopen(infilename, "rb");
    gzFile outfile = gzopen(outfilename, "wb");
    if (!infile || !outfile) return -1;

    char inbuffer[128];
    int num_read = 0;
    unsigned long total_read = 0, total_wrote = 0;
    while ((num_read = fread(inbuffer, 1, sizeof(inbuffer), infile)) > 0) {
        total_read += num_read;
        gzwrite(outfile, inbuffer, num_read);
    }
    fclose(infile);
    gzclose(outfile);

    printf("Read %ld bytes, Wrote %ld bytes, Compression factor %4.2f%%\n",
       total_read, file_size(outfilename),
       (1.0-file_size(outfilename)*1.0/total_read)*100.0
    );
}

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

    // ENCODER STUP
    PointCloudGridEncoder encoder;
    // settings should match './grid_log_info.txt'
    BoundingBox bb(Vec<float>(-1.0f,0.05f,-1.0f), Vec<float>(1.0f,2.2f,1.0f));
    encoder.settings.grid_precision = GridPrecisionDescriptor(
            Vec8(8,8,8), // dimensions
            bb,
            Vec<BitCount>(BIT_8,BIT_8,BIT_8), // default point encoding
            Vec<BitCount>(BIT_8,BIT_8,BIT_8)  // default color encoding
    );

    // READ RAW DATA FROM FILE
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
    std::cout << "RAW VOXEL data (parsed from file) \n";
    std::cout << "  > voxel count " << v_raw.size() << std::endl;


    Measure t;

    // float avg_clr[4] = {0.0f,0.0f,0.0f,0.0f};
    // float avg_pos[3] = {0.0f,0.0f,0.0f};
    // int skipped = 0;
    // for(auto voxel:v_raw) {
    //     avg_clr[0] += (static_cast<float>(voxel.color_rgba[0]));
    //     avg_clr[1] += (static_cast<float>(voxel.color_rgba[1]));
    //     avg_clr[2] += (static_cast<float>(voxel.color_rgba[2]));
    //     avg_clr[3] += (static_cast<float>(voxel.color_rgba[3]));
    //     if(bb.contains(voxel.pos)) {
    //         avg_pos[0] += voxel.pos[0];
    //         avg_pos[1] += voxel.pos[1];
    //         avg_pos[2] += voxel.pos[2];
    //     }
    //     else {
    //         ++skipped;
    //     }
    // }
    // avg_clr[0] /= v_raw.size();
    // avg_clr[1] /= v_raw.size();
    // avg_clr[2] /= v_raw.size();
    // avg_clr[3] /= v_raw.size();
    // avg_pos[0] /= (v_raw.size() - skipped);
    // avg_pos[1] /= (v_raw.size() - skipped);
    // avg_pos[2] /= (v_raw.size() - skipped);
    //
    // std::cout << "  > avg color "
    //           << avg_clr[0] << ","
    //           << avg_clr[1] << ","
    //           << avg_clr[2] << ","
    //           << avg_clr[3] << std::endl;
    // std::cout << "  > avg pos "
    //           << avg_pos[0] << ","
    //           << avg_pos[1] << ","
    //           << avg_pos[2] << std::endl;

    zmq::message_t msg_v_raw = encoder.encode(v_raw);
    unsigned long sizeDataCompressed  = (msg_v_raw.size() * 1.1) + 12;
    unsigned char* dataCompressed = (unsigned char*)malloc(sizeDataCompressed);

    std::cout << "before compress " << sizeDataCompressed << std::endl;
    t.startWatch();
    int z_result = compress(dataCompressed, &sizeDataCompressed, (unsigned char*) msg_v_raw.data(), msg_v_raw.size());
    std::cout << "Compression took: " << t.stopWatch() << " ms" << std::endl;
    std::cout << "after compress " << sizeDataCompressed << std::endl;

    switch( z_result )
    {
    case Z_OK:
        printf("***** SUCCESS! *****\n");
        break;

    case Z_MEM_ERROR:
        printf("out of memory\n");
        exit(1);    // quit.
        break;

    case Z_BUF_ERROR:
        printf("output buffer wasn't large enough!\n");
        exit(1);    // quit.
        break;
    }
    std::cout << "First Memcpy" << std::endl;
    zmq::message_t msg_v_compressed(sizeDataCompressed);
    memcpy((unsigned char*) msg_v_compressed.data(), (const unsigned char*) dataCompressed, sizeDataCompressed);
    std::cout << "First Memcpy finished" << std::endl;

    unsigned long sizeDataUncompressed = msg_v_raw.size();
    zmq::message_t msg_v_uncompressed(sizeDataUncompressed);

    std::cout << "before uncompress " << sizeDataUncompressed << std::endl;
    t.startWatch();
    z_result = uncompress((unsigned char*) msg_v_uncompressed.data(), &sizeDataUncompressed, (unsigned char*) msg_v_compressed.data(), msg_v_compressed.size());
    std::cout << "Uncompression took: " << t.stopWatch() << " ms" << std::endl;
    std::cout << "after uncompress " << sizeDataUncompressed << std::endl;


    switch( z_result )
    {
    case Z_OK:
        printf("***** SUCCESS! *****\n");
        break;

    case Z_MEM_ERROR:
        printf("out of memory\n");
        exit(1);    // quit.
        break;

    case Z_BUF_ERROR:
        printf("output buffer wasn't large enough!\n");
        exit(1);    // quit.
        break;
    }

    // std::cout << "Second Memcpy" << std::endl;
    // // zmq::message_t msg_v_uncompressed(sizeDataUncompressed);
    // // memcpy((unsigned char*) msg_v_uncompressed.data(), (const unsigned char*) dataUncompressed, sizeDataUncompressed);
    // std::cout << "Second Memcpy finished" << std::endl;

    std::vector<UncompressedVoxel> msg_v_final;
    encoder.decode(msg_v_uncompressed, &msg_v_final);
    std::cout << "DECODED message (encoded using raw voxels)\n";
    std::cout << "  > voxel count " << msg_v_final.size() << std::endl;


    // std::vector<UncompressedVoxel> msg_v_raw_decoded;
    // encoder.decode(msg_v_raw, &msg_v_raw_decoded);
    // std::cout << "DECODED message (encoded using raw voxels)\n";
    // std::cout << "  > voxel count " << msg_v_raw_decoded.size() << std::endl;

    // avg_clr[0] = 0.0f;
    // avg_clr[1] = 0.0f;
    // avg_clr[2] = 0.0f;
    // avg_clr[3] = 0.0f;
    // avg_pos[0] = 0.0f;
    // avg_pos[1] = 0.0f;
    // avg_pos[2] = 0.0f;
    // for(auto voxel:msg_v_raw_decoded) {
    //     avg_clr[0] += (static_cast<float>(voxel.color_rgba[0]));
    //     avg_clr[1] += (static_cast<float>(voxel.color_rgba[1]));
    //     avg_clr[2] += (static_cast<float>(voxel.color_rgba[2]));
    //     avg_clr[3] += (static_cast<float>(voxel.color_rgba[3]));
    //     avg_pos[0] += voxel.pos[0];
    //     avg_pos[1] += voxel.pos[1];
    //     avg_pos[2] += voxel.pos[2];
    // }
    // avg_clr[0] /= msg_v_raw_decoded.size();
    // avg_clr[1] /= msg_v_raw_decoded.size();
    // avg_clr[2] /= msg_v_raw_decoded.size();
    // avg_clr[3] /= msg_v_raw_decoded.size();
    // avg_pos[0] /= msg_v_raw_decoded.size();
    // avg_pos[1] /= msg_v_raw_decoded.size();
    // avg_pos[2] /= msg_v_raw_decoded.size();
    //
    // std::cout << "  > avg color "
    //           << avg_clr[0] << ","
    //           << avg_clr[1] << ","
    //           << avg_clr[2] << ","
    //           << avg_clr[3] << std::endl;
    // std::cout << "  > avg pos "
    //           << avg_pos[0] << ","
    //           << avg_pos[1] << ","
    //           << avg_pos[2] << std::endl;

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
