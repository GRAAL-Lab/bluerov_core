#include <signal.h>
#include <unistd.h>
#include <VirtualFrame.h>
#include <iostream>
#include <thread>

#include "ctrl_toolbox_internal/Futils.h"

volatile sig_atomic_t stop;

void inthand(int signum)
{
    stop = 1;
    std::cout << "\nBye byeeee! (signum: " << signum << ")" << std::endl;
}

int main(int, char**)
{

    signal(SIGINT, inthand);

    Eigen::TransfMatrix wTt, wTg, wTg_virtual;
    wTg.SetTransl(Eigen::Vector3d(10, 0, 0));
    wTg.SetRotMatrix((rml::EulerRPY(M_PI, 0, 0)).ToRotMatrix());
    ctb::VirtualFrame vf;

    vf.SetGain(1.2);
    vf.SetSampleTime(0.1);
    vf.SetMaximumAllowedDistance(0.1);

    futils::Timer printTimer;
    printTimer.Start();

    //vf.ResetState(wTt);

    while (!stop) {

        vf.Compute(wTt, wTg, wTg_virtual);

        if (printTimer.GetCurrentLapTime() > 0.1) {
            printTimer.Lap();
            futils::PrettyPrint(wTg_virtual, "wTt_virtual");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
