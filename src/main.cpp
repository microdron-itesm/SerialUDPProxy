#include <boost/program_options.hpp>
#include <iostream>
#include <fmt/format.h>
#include <stdexcept>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
extern "C"{
#include <UDP.h>
}
#include <thread>

using namespace boost::program_options;

int main(int argc, const char *argv[]) {
    try {
        options_description desc{"Options"};
        desc.add_options()
                ("help,h", "Help screen")
                ("sendPort", value<uint16_t>()->default_value(15001), "Port to send on UDP")
                ("recvPort", value<uint16_t>()->default_value(15000), "Port to recv on UDP")
                ("serialPort", value<std::string>()->required(), "Serial port to use");

        variables_map vm;
        store(parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")){
            std::cout << desc << '\n';
        }

        notify(vm);


        uint16_t sendPort = vm["sendPort"].as<uint16_t>();
        uint16_t recvPort = vm["recvPort"].as<uint16_t>();
        std::string serialPort = vm["serialPort"].as<std::string>();

        std::cout <<
            fmt::format("Using SendPort: {}, RecvPort: {}, on serial port: {}", sendPort, recvPort, serialPort)
            << std::endl;

        int port = open(serialPort.c_str(), O_RDWR);
        if(port < 0){
            std::cerr << fmt::format("Error on open, {}, {}", errno, strerror(errno)) << std::endl;
            throw std::runtime_error("Error on serial port open");
        }

        struct termios tty{};
        if(tcgetattr(port, &tty) != 0) {
            std::cerr << fmt::format("Error on tcgetattr, {}, {}", errno, strerror(errno)) << std::endl;
            throw std::runtime_error("Error on tcgetattr");
        }

        tty.c_lflag &= ~ECHO; // Disable echo
        tty.c_lflag &= ~ECHOE; // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo

        tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl

        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR;

        tty.c_cc[VTIME] = 255;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        tty.c_cc[VMIN] = 0;

        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);

        if (tcsetattr(port, TCSANOW, &tty) != 0) {
            std::cerr << fmt::format("Error on tcsetattr, {}, {}", errno, strerror(errno)) << std::endl;
            throw std::runtime_error("Error on tcsetattr");
        }

        udp_conn_data data;
        udp_conn_open(&data, sendPort, recvPort);

        uint8_t buf[2046];
        for(;;){
            auto size = udp_conn_recv(&data, buf, sizeof(buf));
            write(port, buf, size);
            memset(buf, 0, sizeof(buf));
            auto n = read(port, &buf, sizeof(buf));
            udp_conn_send(&data, buf, n);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    catch (const error &ex) {
        std::cerr << ex.what() << '\n';
    }
}