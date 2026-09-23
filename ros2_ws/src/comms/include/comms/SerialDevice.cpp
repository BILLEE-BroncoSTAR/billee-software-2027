#include <termios.h>
#include <unistd.h>
#include <sys/fcntl.h>
#include <cstring>
#include <iostream>
#include <cerrno>

class SerialDevice{
    private:
        char* deviceName;
        int fd;
        speed_t baudRate;

    public:
        SerialDevice(char* deviceName) : deviceName(deviceName), fd(-1), baudRate(B9600){}
        ~SerialDevice() {
            delete deviceName;
            if (fd != -1)
                close(fd);
        }

        bool openDevice(){
            if (fd != -1)
                close(fd);

            fd = open(deviceName, O_RDONLY);

            if(fd < 0){
                std::cout << "Could not open serial device: " << std::strerror(errno) << std::endl;
                return false;
            }

            struct termios deviceSettings;

            // configure the device settings
            tcgetattr(fd, &deviceSettings);
            cfmakeraw(&deviceSettings);
            cfsetspeed(&deviceSettings, baudRate);
            deviceSettings.c_iflag &= ~(IXON | IXOFF | IGNCR);
            deviceSettings.c_iflag |= ICRNL; 
            // Enable reading the device and disable modem pins (generally unused on most devices)
            deviceSettings.c_cflag |= (CREAD | CLOCAL);
            deviceSettings.c_cflag &= ~CSTOPB;

            deviceSettings.c_oflag = 0;
            deviceSettings.c_lflag = 0;

            deviceSettings.c_cc[VTIME] = 5;
            deviceSettings.c_cc[VMIN] = 0;


            if(tcsetattr(fd, TCSANOW, &deviceSettings)){
                std::cout << "Could not configure serial device: " << std::strerror(errno) << std::endl;
                return false;
            }

            tcflush(fd, TCIFLUSH);
            return true;
        }

        int readMsg(){
            char msg[256];
            size_t numBytes = read(fd, msg, 256);

            if(numBytes == 0)
                return -1;
            else{
                msg[255] = '\0';
                return atoi(msg);
            }
            
        }
};