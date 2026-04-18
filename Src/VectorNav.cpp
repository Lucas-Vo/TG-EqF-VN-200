#include "VectorNav.hpp"

#include <chrono>
#include <iostream>
#include <thread>

#if __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#endif

using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::math;

namespace
{
constexpr std::uint32_t kConfigureRetryCount = 3U;
constexpr auto kReconnectSettleDelay = std::chrono::milliseconds(500);
constexpr auto kSerialPulseHighDelay = std::chrono::milliseconds(150);
constexpr auto kSerialPulseLowDelay = std::chrono::milliseconds(150);

#if __linux__ || __APPLE__ || __CYGWIN__ || __QNXNTO__
bool baudrateToTermios(std::uint32_t baudrate, speed_t& out)
{
    switch (baudrate)
    {
        case 9600U:
            out = B9600;
            return true;
        case 19200U:
            out = B19200;
            return true;
        case 38400U:
            out = B38400;
            return true;
        case 57600U:
            out = B57600;
            return true;
        case 115200U:
            out = B115200;
            return true;
#if !defined(__QNXNTO__)
        case 230400U:
            out = B230400;
            return true;
#if !defined(__APPLE__)
        case 460800U:
            out = B460800;
            return true;
        case 921600U:
            out = B921600;
            return true;
#endif
#endif
        default:
            return false;
    }
}

void pulseSerialPort(const std::string& portName, std::uint32_t baudrate)
{
    const int fd = ::open(portName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1)
    {
        std::cerr << "Serial pulse open failed: " << std::strerror(errno) << '\n';
        return;
    }

    speed_t speed = B115200;
    baudrateToTermios(baudrate, speed);

    termios settings{};
    if (tcgetattr(fd, &settings) == 0)
    {
        cfmakeraw(&settings);
        cfsetispeed(&settings, speed);
        cfsetospeed(&settings, speed);
        settings.c_cflag |= CLOCAL | CREAD | HUPCL;
#ifdef CRTSCTS
        settings.c_cflag &= ~CRTSCTS;
#endif
        settings.c_cflag &= ~PARENB;
        settings.c_cflag &= ~CSTOPB;
        settings.c_cflag &= ~CSIZE;
        settings.c_cflag |= CS8;
        tcsetattr(fd, TCSANOW, &settings);
    }

    tcflush(fd, TCIOFLUSH);

#if defined(TIOCMBIS) && defined(TIOCMBIC) && defined(TIOCM_DTR) && defined(TIOCM_RTS)
    int modemBits = TIOCM_DTR | TIOCM_RTS;
    if (ioctl(fd, TIOCMBIS, &modemBits) == 0)
    {
        std::this_thread::sleep_for(kSerialPulseHighDelay);
        ioctl(fd, TIOCMBIC, &modemBits);
        std::this_thread::sleep_for(kSerialPulseLowDelay);
    }
#endif

    ::close(fd);
    std::this_thread::sleep_for(kReconnectSettleDelay);
}
#else
void pulseSerialPort(const std::string&, std::uint32_t)
{
}
#endif
}

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

    const auto connectForStreaming = [this, &portName, streamBaud]() -> bool
    {
        bool connectedAtStreamBaud = false;
        if (streamBaud != initialBaud_)
        {
            std::cout << "Connecting to " << portName << " at " << streamBaud << "...\n";
            connectedAtStreamBaud = connect(portName, streamBaud);
        }

        if (connectedAtStreamBaud)
        {
            return true;
        }

        std::cout << "Connecting to " << portName << " at " << initialBaud_ << "...\n";
        if (!connect(portName, initialBaud_))
        {
            return false;
        }

        if (streamBaud != initialBaud_)
        {
            std::cout << "Changing baud to " << streamBaud << "...\n";
            if (!changeBaud(streamBaud))
            {
                return false;
            }
        }

        return true;
    };

    pulseSerialPort(portName, initialBaud_);

    if (!connectForStreaming())
    {
        std::cerr << "Could not connect to sensor\n";
        return false;
    }

    const std::uint16_t rateDivisor = static_cast<std::uint16_t>(800U / asyncDataOutputFrequency);
    std::cout << "Configuring Binary Output 1 for " << asyncDataOutputFrequency << " Hz...\n";
    bool configured = false;
    for (std::uint32_t attempt = 1U; attempt <= kConfigureRetryCount; ++attempt)
    {
        if (configureBinaryOutput1(rateDivisor))
        {
            configured = true;
            break;
        }

        if (attempt == kConfigureRetryCount)
        {
            break;
        }

        std::cerr << "Retrying Binary Output 1 after serial pulse (attempt "
                  << (attempt + 1U) << "/" << kConfigureRetryCount << ")...\n";
        disconnect();
        std::this_thread::sleep_for(kReconnectSettleDelay);
        pulseSerialPort(portName, streamBaud);

        if (!connectForStreaming())
        {
            std::cerr << "Reconnect before Binary Output 1 retry failed\n";
        }
    }

    if (!configured)
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
