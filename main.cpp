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
constexpr const char* kEqFLabelBnuEqNu = "TG-EqF b_nu = nu";
constexpr const char* kEqFLabelBnuNeqNu = "TG-EqF b_nu != nu";
constexpr const char* kEqFLogPathBnuEqNu = "log/TGEqFEstimate_b_nu_eq_nu.csv";
constexpr const char* kEqFLogPathBnuNeqNu = "log/TGEqFEstimate_b_nu_neq_nu.csv";

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
    TGEqF tgEqFBnuEqNu(false);
    TGEqF tgEqFBnuNeqNu(true);

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
        tgEqFBnuEqNu.IMUpropagagte(result.gyroData, result.accData, result.time);
        tgEqFBnuNeqNu.IMUpropagagte(result.gyroData, result.accData, result.time);

        // GNSS update
        if (result.hasGnssMeasurement)
        {
            tgEqFBnuEqNu.GnssUpdate(result.gnssPosData, result.gnssVelData);
            tgEqFBnuNeqNu.GnssUpdate(result.gnssPosData, result.gnssVelData);
        }

        // baro update
        // EqF.BaroUpdate(result.baroData);
        
        // mag update
        tgEqFBnuEqNu.MagUpdate(result.magData);
        tgEqFBnuNeqNu.MagUpdate(result.magData);
        
        const EqFOutput outputBnuEqNu = tgEqFBnuEqNu.GetEqFOutput();
        const EqFOutput outputBnuNeqNu = tgEqFBnuNeqNu.GetEqFOutput();
        printTGEqFEstimate(outputBnuEqNu, data, kEqFLabelBnuEqNu);
        logTGEqFEstimate(outputBnuEqNu, data, kEqFLogPathBnuEqNu);
        printTGEqFEstimate(outputBnuNeqNu, data, kEqFLabelBnuNeqNu);
        logTGEqFEstimate(outputBnuNeqNu, data, kEqFLogPathBnuNeqNu);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return 0;
}
