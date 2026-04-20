#include "VectorNav.hpp"
#include "EqFalgo.hpp"
#include "EqFparser.hpp"
#include "EqFlogger.hpp"

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
    const Vec3 m = {-0.362255, -0.012619, 0.931994};
    TGEqF TgEqF = TGEqF(m);

    // logging logic

    while (gRunning)
    {
        // fetch data
        vectornavData data{};
        if (!vn.latest(data))
        {
            break;
        }
        printVNEstimate(data);
        logVNEstimate(data);

        /* BEGIN Filter Logic */

        // parse to EqF friendly data
        EqFparserResult result = EqFparser(data);
        printRaw(result);
        printMeasurements(result, data, m);
        logMeasurements(result, data, m);

        // IMU propogate
        TgEqF.IMUpropagagte(result.gyroData, result.accData, result.time);

        // GNSS update
        if (result.hasGnssMeasurement)
        {
            TgEqF.GnssUpdate(result.gnssPosData, result.gnssVelData);
        }

        // baro update
        // EqF.BaroUpdate(result.baroData);
        
        // mag update
        TgEqF.MagUpdate(result.magData);
        
        EqFOutput output = TgEqF.GetEqFOutput();
        printTGEqFEstimate(output, data);
        logTGEqFEstimate(output, data);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return 0;
}
