#include "VectorNav.hpp"

#include <iostream>

using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::math;

const char* gpsFixLabel(std::uint8_t fix)
{
    switch (fix)
    {
        case 0U:
            return "NoFix";
        case 1U:
            return "TimeOnly";
        case 2U:
            return "2D";
        case 3U:
            return "3D";
        case 4U:
            return "SBAS";
        case 7U:
            return "RtkFloat";
        case 8U:
            return "RtkFix";
        default:
            return "UnknownOrExtended";
    }
}

bool gpsFixHasPosition(std::uint8_t fix)
{
    return (fix == 3U) || (fix == 4U);
}

std::ostream& operator<<(std::ostream& os, const vectornavData& data)
{
    os << "vectornavData{"
    //    << "TimeStartup=" << data.TimeStartup
       << ", Fix=" << gpsFixLabel(data.Fix)
       << "(" << static_cast<unsigned int>(data.Fix) << ")"
       << ", PosEcef=[" << data.GnssPosEcef(0) << ", " << data.GnssPosEcef(1) << ", " << data.GnssPosEcef(2) << "]"
    //    << ", Temp=" << data.Temp
    //    << ", Pres=" << data.Pres
    //    << ", Mag=[" << data.Mag(0) << ", " << data.Mag(1) << ", " << data.Mag(2) << "]"
    //    << ", UncompGyro=[" << data.UncompGyro(0) << ", " << data.UncompGyro(1) << ", " << data.UncompGyro(2) << "]"
    //    << ", UncompAccel=[" << data.UncompAccel(0) << ", " << data.UncompAccel(1) << ", " << data.UncompAccel(2) << "]"
       << "}";
    return os;
}

VectorNav::~VectorNav()
{
    disconnect();
}

bool VectorNav::init(const std::string& portName,
                     std::uint32_t streamBaud,
                     std::uint32_t asyncDataOutputFrequency)
{
    if ((asyncDataOutputFrequency == 0U) || (asyncDataOutputFrequency > 800U))
    {
        std::cerr << "Invalid asyncDataOutputFrequency: " << asyncDataOutputFrequency << '\n';
        return false;
    }

    bool connectedAtStreamBaud = false;
    if (streamBaud != initialBaud_)
    {
        std::cout << "Connecting to " << portName << " at " << streamBaud << "...\n";
        connectedAtStreamBaud = connect(portName, streamBaud);
    }

    if (!connectedAtStreamBaud)
    {
        std::cout << "Connecting to " << portName << " at " << initialBaud_ << "...\n";
        if (!connect(portName, initialBaud_))
        {
            std::cerr << "Could not connect to sensor\n";
            return false;
        }

        if (streamBaud != initialBaud_)
        {
            std::cout << "Changing baud to " << streamBaud << "...\n";
            if (!changeBaud(streamBaud))
            {
                std::cerr << "Could not change baud\n";
                return false;
            }
        }
    }

    const std::uint16_t rateDivisor = static_cast<std::uint16_t>(800U / asyncDataOutputFrequency);
    std::cout << "Configuring Binary Output 1 for " << asyncDataOutputFrequency << " Hz...\n";
    if (!configureBinaryOutput1(rateDivisor))
    {
        std::cerr << "Could not configure Binary Output 1\n";
        return false;
    }

    std::cout << "Streaming at " << asyncDataOutputFrequency << " Hz. Ctrl+C to stop.\n";
    return true;
}

bool VectorNav::configureBinaryOutput1(std::uint16_t rateDivisor)
{
    if (!connected_.load())
    {
        return false;
    }

    try
    {

        // Disable ASCII async on port 1 so it doesn't mix with binary output.
        sensor_.writeAsyncDataOutputType(VNOFF, 1, true);

        BinaryOutputRegister bor(
            ASYNCMODE_PORT1,
            rateDivisor,
            COMMONGROUP_NONE,
            TIMEGROUP_TIMESTARTUP,
            static_cast<ImuGroup>(IMUGROUP_UNCOMPACCEL |
                                  IMUGROUP_UNCOMPGYRO |
                                  IMUGROUP_TEMP |
                                  IMUGROUP_PRES |
                                  IMUGROUP_MAG),
            static_cast<GpsGroup>(GPSGROUP_FIX |
                                  GPSGROUP_POSECEF),
            static_cast<AttitudeGroup>(ATTITUDEGROUP_DCM |
                                       ATTITUDEGROUP_ACCELNED),
            static_cast<InsGroup>(INSGROUP_POSECEF |
                                  INSGROUP_VELNED |
                                  INSGROUP_POSU |
                                  INSGROUP_VELU),
            GPSGROUP_NONE);

        sensor_.writeBinaryOutput1(bor, true);
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "VectorNav configure failed: " << e.what() << '\n';
        return false;
    }
}

bool VectorNav::connect(const std::string& portName, std::uint32_t baudrate)
{
    try
    {
        sensor_.connect(portName, baudrate);
        sensor_.registerAsyncPacketReceivedHandler(this, &VectorNav::asyncPacketHandler);
        connected_.store(true);
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "VectorNav connect failed: " << e.what() << '\n';
        connected_.store(false);
        return false;
    }
}

void VectorNav::disconnect()
{
    if (!connected_.load())
    {
        return;
    }

    try
    {
        sensor_.unregisterAsyncPacketReceivedHandler();
    }
    catch (...)
    {
    }

    try
    {
        sensor_.disconnect();
    }
    catch (...)
    {
    }

    connected_.store(false);
}

bool VectorNav::changeBaud(std::uint32_t newBaudrate)
{
    if (!connected_.load())
    {
        return false;
    }

    try
    {
        sensor_.changeBaudRate(newBaudrate);
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "VectorNav changeBaud failed: " << e.what() << '\n';
        return false;
    }
}

bool VectorNav::latest(vectornavData& out) const
{
    if (!connected_.load())
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(dataMutex_);
    out = latestData_;
    return true;
}

void VectorNav::asyncPacketHandler(void* userData,
                                      vn::protocol::uart::Packet& packet,
                                      size_t /*runningIndex*/)
{
    auto* self = static_cast<VectorNav*>(userData);
    if (self)
    {
        self->handlePacket(packet);
    }
}
void VectorNav::handlePacket(vn::protocol::uart::Packet& packet)
{
    if (packet.type() != Packet::TYPE_BINARY)
    {
        return;
    }

    const auto expectedImu =
        static_cast<ImuGroup>(IMUGROUP_UNCOMPACCEL |
                              IMUGROUP_UNCOMPGYRO |
                              IMUGROUP_TEMP |
                              IMUGROUP_PRES |
                              IMUGROUP_MAG);

    const auto expectedGps =
        static_cast<GpsGroup>(GPSGROUP_FIX |
                              GPSGROUP_POSECEF);

    const auto expectedAttitude =
        static_cast<AttitudeGroup>(ATTITUDEGROUP_DCM |
                                   ATTITUDEGROUP_ACCELNED);

    const auto expectedIns =
        static_cast<InsGroup>(INSGROUP_POSECEF |
                              INSGROUP_VELNED |
                              INSGROUP_POSU |
                              INSGROUP_VELU);

    if (!packet.isCompatible(
            COMMONGROUP_NONE,
            TIMEGROUP_TIMESTARTUP,
            expectedImu,
            expectedGps,
            expectedAttitude,
            expectedIns,
            GPSGROUP_NONE))
    {
        return;
    }

    vectornavData d{};

    d.TimeStartup = packet.extractUint64();

    {
        const vec3f v = packet.extractVec3f();
        d.UncompAccel << v.x, v.y, v.z;
    }
    {
        const vec3f v = packet.extractVec3f();
        d.UncompGyro << v.x, v.y, v.z;
    }

    d.Temp = packet.extractFloat();
    d.Pres = packet.extractFloat();

    {
        const vec3f v = packet.extractVec3f();
        d.Mag << v.x, v.y, v.z;
    }

    // GPSGROUP_FIX | GPSGROUP_POSECEF
    d.Fix = packet.extractUint8();

    {
        const vec3d v = packet.extractVec3d();
        d.GnssPosEcef << v.x, v.y, v.z;
    }

    {
        const mat3f v = packet.extractMat3f();
        d.Dcm <<
            v.e00, v.e01, v.e02,
            v.e10, v.e11, v.e12,
            v.e20, v.e21, v.e22;
    }

    {
        const vec3f v = packet.extractVec3f();
        d.AccelNed << v.x, v.y, v.z;
    }

    {
        const vec3d v = packet.extractVec3d();
        d.InsPosEcef << v.x, v.y, v.z;
    }

    {
        const vec3f v = packet.extractVec3f();
        d.InsVelNed << v.x, v.y, v.z;
    }

    d.InsPosU = packet.extractFloat();
    d.InsVelU = packet.extractFloat();

    std::lock_guard<std::mutex> lock(dataMutex_);
    latestData_ = d;
}
