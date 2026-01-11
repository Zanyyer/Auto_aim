#ifndef IO_SERIAL_HPP
#define IO_SERIAL_HPP

#include <string>
#include <cstdint>

namespace io {

class Serial {
public:
    Serial(const std::string& port, int baudrate);
    ~Serial();

    bool open();
    void close();
    bool isOpened() const;

    bool send(const uint8_t* data, size_t size);

private:
    int fd_;
    std::string port_;
    int baudrate_;
};

} // namespace io
#endif // IO_SERIAL_HPP
