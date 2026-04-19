#pragma once

#include <Eigen/Dense>
#include <atomic>
#include <cstdint>
#include <iosfwd>
#include <mutex>
#include <string>

#include "vn/sensors.h"

using Vec3f = Eigen::Matrix<float, 3, 1>;
using Vec3d = Eigen::Matrix<double, 3, 1>;
using Mat3f = Eigen::Matrix<float, 3, 3>;

struct vectornavData
{
    // Common Group
    std::uint64_t TimeStartup = 0; // ns

    // IMU Group
    Vec3f UncompAccel = Vec3f::Zero(); // ms-2
    Vec3f UncompGyro = Vec3f::Zero(); // rad/s
    float Temp = 0.0F; // deg C
    float Pres = 0.0F; // kPa
    Vec3f Mag = Vec3f::Zero(); // gauss
    Vec3f Accel = Vec3f::Zero(); // ms-2 compensated body acceleration

    // GNSS Group
    std::uint8_t Fix = 0; // GPS fix quality
    Vec3d GnssPosEcef = Vec3d::Zero(); // m ECEF
    Vec3f GnssVelNed = Vec3f::Zero(); // ms-1 NED

    // Attitude Group
    Mat3f Dcm = Mat3f::Zero(); // attitude DCM
    
    // INS Group
    Vec3d InsPosEcef = Vec3d::Zero(); // m ECEF
    Vec3f InsVelNed = Vec3f::Zero(); // ms-1 NED
    float InsPosU = 0.0F; // m
    float InsVelU = 0.0F; // ms-1
};

const char* gpsFixLabel(std::uint8_t fix);
bool gpsFixHasPosition(std::uint8_t fix);

std::ostream& operator<<(std::ostream& os, const vectornavData& data);

class VectorNav
{
public:
    VectorNav() = default;
    ~VectorNav();

    VectorNav(const VectorNav&) = delete;
    VectorNav& operator=(const VectorNav&) = delete;

    bool init(const std::string& portName,
              std::uint32_t streamBaud,
              std::uint32_t asyncDataOutputFrequency);
    
    bool latest(vectornavData& out) const;
    
    private:
    
    bool connect(const std::string& portName, std::uint32_t baudrate);
    void disconnect();
    bool changeBaud(std::uint32_t newBaudrate);
    bool configureBinaryOutput1(std::uint16_t rateDivisor);

    static void asyncPacketHandler(void* userData,
                                   vn::protocol::uart::Packet& packet,
                                   size_t runningIndex);

    void handlePacket(vn::protocol::uart::Packet& packet);

private:
    vn::sensors::VnSensor sensor_;
    std::atomic<bool> connected_{false};
    static constexpr std::uint32_t initialBaud_ = 115200U;

    mutable std::mutex dataMutex_;
    vectornavData latestData_{};
};
