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
constexpr const char* kEqFLabelReset = "TG-EqF reset";
constexpr const char* kEqFLabelNoReset = "TG-EqF no reset";
constexpr const char* kEqFLogPathReset = "log/TGEqFEstimate_reset.csv";
constexpr const char* kEqFLogPathNoReset = "log/TGEqFEstimate_no_reset.csv";
constexpr const char* kEqFResetColor = "\033[94m";
constexpr const char* kEqFNoResetColor = "\033[96m";

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
    const std::uint32_t asyncDataOutputFrequency = 5U;
    if (!vn.init(port, streamBaud, asyncDataOutputFrequency))
    {
        return 1;
    }

    // filter logic
    TGEqF tgEqFReset(true);
    TGEqF tgEqFNoReset(false);

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
        printMeasurements(result, data);
        logMeasurements(result, data);

        // IMU propogate
        tgEqFReset.IMUpropagagte(result.gyroData, result.accData, result.time);
        tgEqFNoReset.IMUpropagagte(result.gyroData, result.accData, result.time);

        // GNSS update
        if (result.hasGnssMeasurement)
        {
            tgEqFReset.GnssUpdate(result.gnssPosData, result.gnssVelData);
            tgEqFNoReset.GnssUpdate(result.gnssPosData, result.gnssVelData);
        }

        // baro update
        // EqF.BaroUpdate(result.baroData);
        
        // mag update
        tgEqFReset.MagUpdate(result.magData);
        tgEqFNoReset.MagUpdate(result.magData);
        
        const EqFOutput outputReset = tgEqFReset.GetEqFOutput();
        const EqFOutput outputNoReset = tgEqFNoReset.GetEqFOutput();
        const double frobeniusNorm = (outputReset.Sigma - outputNoReset.Sigma).norm();

        printTGEqFEstimate(outputReset, data, kEqFLabelReset, kEqFResetColor);
        logTGEqFEstimate(outputReset, data, kEqFLogPathReset);
        printTGEqFEstimate(outputNoReset, data, kEqFLabelNoReset, kEqFNoResetColor);
        logTGEqFEstimate(outputNoReset, data, kEqFLogPathNoReset);
        logFrobeniusNorm(result.time, frobeniusNorm);

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return 0;
}
