#include "VectorNav.hpp"
#include "EqFalgo.hpp"
#include "utils.hpp"

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <thread>

namespace
{
volatile std::sig_atomic_t gRunning = 1;

void signalHandler(int)
{
    gRunning = 0;
}
}

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;
    
    // ctrl + C handling
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    // sensor logic
    VectorNav vn;
    const std::string port = "/dev/ttyUSB0";
    // const std::uint32_t streamBaud = 921600U;
    const std::uint32_t streamBaud = 115200U;
    // const std::uint32_t asyncDataOutputFrequency = 200U;
    const std::uint32_t asyncDataOutputFrequency = 50U;
    if (!vn.init(port, streamBaud, asyncDataOutputFrequency))
    {
        return 1;
    }

    // filter logic
    EqFalgo EqF = EqFalgo();

    // logging logic

    while (gRunning)
    {
        // sensor logic
        vectornavData data{};
        if (!vn.latest(data))
        {
            break;
        }

        // filter logic

        // parse to EqF friendly data
        EqFparserResult result = EqFparser(data);
        printINSEstimate(data);

        // IMU propogate
        EqF.IMUpropagagte(result.gyroData, result.accData, result.time);

        // GNSS update
        if (result.gnssData != Vec3::Zero())
        {
            EqF.GnssUpdate(result.gnssData);
        }
        
        // mag update
        EqF.MagUpdate(result.magData);
        
        EqFOutput output = EqF.GetEqFOutput();
        printEqFEstimate(output, data);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}
