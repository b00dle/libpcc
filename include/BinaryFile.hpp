#ifndef LIBPCC_BINARYFILE_HPP
#define LIBPCC_BINARYFILE_HPP

#include <string>
#include <zmq.hpp>

/**
 * Wrapper for binary file read and write functionality.
 * Can be used to store and extract
 * compressed and uncompressed point cloud data.
*/
class BinaryFile {
public:
    BinaryFile();
    BinaryFile(const char* data, size_t size);
    explicit BinaryFile(zmq::message_t& msg);
    ~BinaryFile();

    zmq::message_t get() const;
    const char* getData() const;
    long getSize() const;

    void set(const char *data, size_t size);
    void set(zmq::message_t& msg);

    void copy(char* data, size_t size) const;
    void copy(char* data) const;

    static bool read(const std::string& file_path, char*& data, long& size);
    static bool write(const std::string& file_path, const char* data, long size);

    bool read(const std::string& file_path);
    bool write(const std::string& file_path);

private:
    char* data_;
    long size_;
};


#endif //LIBPCC_BINARYFILE_HPP
