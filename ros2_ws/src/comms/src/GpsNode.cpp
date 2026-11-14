#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <cstdint>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "SerialDevice.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "Gps.hpp"

#define VALID_HEADER 0xb562
#define POS_ID 0x0136
#define COV_ID 0x2936
#define HEADER_SZ 4*8
#define POS_SIZE 28
#define COV_SIZE 64 


using namespace std::chrono_literals;

class Gps : public rclcpp::Node{

    public:
        Gps() : Node("gps"){
            //TODO: tune the QOS
            gpsPublisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps", 1);

            hasPos = false;
            hasCov = false;

            pendingPos = new posPayload();
            pendingCov = new covPayload();

            timer_ = this->create_wall_timer(500ms, std::bind(&Gps::timer_callback, this));

            device = new SerialDevice("test");
        }

        ~Gps(){
            delete pendingPos;
            delete pendingCov;
            delete device;
        }

    private:
        posPayload* pendingPos; 
        covPayload* pendingCov; 
        bool hasPos;
        bool hasCov;    
        rclcpp::Time posTs;
        rclcpp::Time covTs;

        SerialDevice* device;
        const int BUF_LEN = 128;

        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gpsPublisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        int tolSec;

        int16_t readMsg(char buf[], int bufLen){

            device->openDevice();

            char headerBuf[HEADER_SZ];
            char* endptr;

            size_t bytesRead = device->readMsg(buf, bufLen);

            if(!bytesRead){
                std::cout << "No Mesage read" << std::endl;
                return -1;
            }

            // construct the header by grabbing the first 4 bytes of the buffer 
            uint32_t header = (static_cast<uint8_t>(buf[0]) << 24) |
                   (static_cast<uint8_t>(buf[1]) << 16) |
                   (static_cast<uint8_t>(buf[2]) << 8)  |
                    static_cast<uint8_t>(buf[3]);
            uint16_t msgHeader = header >> (HEADER_SZ / 2) ;
            uint16_t msgId = header & 0xFFFF; 

            // got valid message
            if(!(msgHeader ^ VALID_HEADER)){
                switch(msgId){
                case COV_ID:
                case POS_ID:
                    return msgId;
                default:
                    std::cout << "Message ID was invalid, got: " << std::hex << msgHeader << std::endl;
                    return -1;
                }
            }
            else{
                std::cout << "Message Header was invalid, got: " << std::hex << msgHeader << std::endl;
                return -1;
            }
        }

        void pollMsgs(){
            char buf[BUF_LEN];
            int16_t msgType = readMsg(buf, BUF_LEN);

            if(msgType != -1){
                switch(msgType){
                case POS_ID:
                    std::memcpy(pendingPos, buf + HEADER_SZ / 8, POS_SIZE);
                    hasPos = true;
                    posTs = this->get_clock()->now();
                    break;
                case COV_ID:
                    std::memcpy(pendingCov, buf + HEADER_SZ / 8, COV_SIZE);
                    hasCov = true;
                    covTs = this->get_clock()->now();
                    break;
                default:
                    break;
                }
            }
        }

        void timer_callback(){
            pollMsgs();

            if(hasPos && hasCov){
                //TODO: play around with the tolerancing for timestamps, there should be a 
                // way to configure the receiver to output messages at regular intervals so 
                // the timestamps are the exact same 
                if (pendingPos->iTow == pendingCov->iTow){
                    auto gpsMsg = constructGpsMsg();
                    gpsPublisher_->publish(gpsMsg);
                    hasPos = hasCov = false;
                }
            }

            auto currentTs = this->get_clock()->now();
            if(
                (hasPos && (currentTs - posTs).seconds() > tolSec) ||
                (hasCov && (currentTs - covTs).seconds() > tolSec)
            )
                hasPos = hasCov = false;
        }

        sensor_msgs::msg::NavSatFix constructGpsMsg(){
            auto gpsMsg = sensor_msgs::msg::NavSatFix();
        
            // conversions, see pg 122 of: https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPS-1.40_InterfaceDescription_UBXDOC-963802114-13138.pdf?utm_content=UBXDOC-963802114-13138
            gpsMsg.longitude = pendingPos->longitude * 1e-7;
            gpsMsg.latitude = pendingPos->latitude * 1e-7;
            gpsMsg.altitude = pendingPos->height * 1e-3;


            // conversions, see pg 113 of: https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPS-1.40_InterfaceDescription_UBXDOC-963802114-13138.pdf?utm_content=UBXDOC-963802114-13138
            if(pendingCov->posCovValid){
                // cov matrix components are ENU in row major order
                gpsMsg.position_covariance[0] = pendingCov->posCovEE;
                gpsMsg.position_covariance[1] = pendingCov->posCovNE;
                gpsMsg.position_covariance[2] = -pendingCov->posCovED;
                gpsMsg.position_covariance[3] = pendingCov->posCovNE;
                gpsMsg.position_covariance[4] = pendingCov->posCovNN;
                gpsMsg.position_covariance[5] = -pendingCov->posCovND;
                gpsMsg.position_covariance[6] = -pendingCov->posCovED;
                gpsMsg.position_covariance[7] = -pendingCov->posCovND;
                gpsMsg.position_covariance[8] = -pendingCov->posCovDD;

                gpsMsg.position_covariance_type = gpsMsg.COVARIANCE_TYPE_KNOWN;
            }
            else
                gpsMsg.position_covariance_type = gpsMsg.COVARIANCE_TYPE_UNKNOWN;

            return gpsMsg;
        }

};

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Gps>());
    rclcpp::shutdown();

    return 0;
}

  //need to have it so only ubx msgs send

  //https://man7.org/linux/man-pages/man3/termios.3.html
  //https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPS-1.40_InterfaceDescription_UBXDOC-963802114-13138.pdf?utm_content=UBXDOC-963802114-13138
  //use ubx-nav-posllh and cov msgs



//TODO: read thru checksum bytes
  