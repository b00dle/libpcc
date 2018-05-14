#include "BinaryFile.hpp"

#include <fstream>
#include <iostream>

BinaryFile::BinaryFile()
    : data_(nullptr)
    , size_(-1)
{}

BinaryFile::BinaryFile(const char *data, size_t size)
    : data_(nullptr)
    , size_(-1)
{
    set(data, size);
}

BinaryFile::BinaryFile(zmq::message_t &msg)
    : data_(nullptr)
    , size_(-1)
{
    set(msg);
}

BinaryFile::~BinaryFile()
{
    if(data_ != nullptr)
        delete [] data_;
}

zmq::message_t BinaryFile::get() const {
    if (size_ == -1)
        return zmq::message_t();

    zmq::message_t msg(static_cast<size_t>(size_));
    copy((char*) msg.data(), msg.size());

    return msg;
}

const char *BinaryFile::getData() const
{
    return data_;
}

long BinaryFile::getSize() const
{
    return size_;
}

void BinaryFile::set(const char *data, size_t size)
{
    if(data_ != nullptr)
        delete [] data_;
    data_ = new char[size];
    memcpy(data_, data, size);
    size_ = size;
}

void BinaryFile::set(zmq::message_t& msg)
{
    set((const char*) msg.data(), msg.size());
}

void BinaryFile::copy(char* data, size_t size) const
{
    memcpy(data, getData(), size);
}

void BinaryFile::copy(char *data) const
{
    if(size_ < 0)
        return;
    return copy(data, static_cast<size_t>(getSize()));
}

bool BinaryFile::read(const std::string &file_path, char *&data, long &size)
{
    std::ifstream file(file_path, std::ifstream::binary);

    file.seekg(0, file.end);
    size = file.tellg();
    file.seekg(0, file.beg);

    if(size == -1)
        return false;

    delete [] data;
    data = new char[size];

    file.read(reinterpret_cast<char*>(data), size);

    return file ? true : false;
}

bool BinaryFile::write(const std::string &file_path, const char *data, long size)
{
    std::ofstream file(file_path, std::ofstream::binary);

    file.write(data, size);

    return file ? true : false;
}

bool BinaryFile::read(const std::string &file_path)
{
    return BinaryFile::read(file_path, data_, size_);
}

bool BinaryFile::write(const std::string &file_path)
{
    return BinaryFile::write(file_path, data_, size_);
}
