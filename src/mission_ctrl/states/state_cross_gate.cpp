#include "mission_ctrl/states/state_cross_gate.hpp"

namespace mission {

namespace states {

    StateCrossGate::StateCrossGate()
    {
        gate = nullptr;
    }

    StateCrossGate::~StateCrossGate() { }

    fsm::retval StateCrossGate::OnEntry()
    {

        fsm::retval ret;

        // temp
        double alt;
        Eigen::Vector3d distanceVector(0, 2, 0);
        ctb::LatLong pos;
        ctb::LocalNED2LatLong(distanceVector, ctb::LatLong(0, 0), pos, alt);
        Buoy gb1, gb2;
        Gate gt;
        gb1.position = ctb::LatLong(0, 0);
        gb2.position = pos;
        if (gt.SetGateBuoys(gb1, gb2)) {
            gate = std::make_shared<Gate>(gt);
        } else {
            return fsm::fail;
        }

        if (gate == nullptr)
            ret = fsm::fail;
        else {
            std::cerr << "Crossing gate...\n";
            ret = genPath();
        }

        return ret;
    }

    fsm::retval StateCrossGate::Execute()
    {
        // if (found) {
        //     taskData_->taskPhases.pop();
        //     std::cerr << "Found!\n";
        //     return fsm_->SetNextState(taskData_->taskPhases.front().first);
        // }
        // double delta = std::fmod((ctrlData->bodyF_angularPosition.Yaw() - previous_bodyF_angularPosition.Yaw()) + 180, 360) - 180;
        // cumulativeAngle += delta;
        // previous_bodyF_angularPosition = ctrlData->bodyF_angularPosition;

        // if (cumulativeAngle > 360 || cumulativeAngle < -360) {
        //     std::cerr << "Not Found!\n";
        //     // return fsm_->SetNextState(taskData_->taskPhases.front().first);
        //     return fsm::fail;
        // }

        // std::cerr << "Searching...\n";
        // std::cerr << "Cumulative angle: " << cumulativeAngle << "\n";

        return fsm::ok;
    }

    fsm::retval StateCrossGate::OnExit()
    {
        path = nullptr;
        return fsm::ok;
    }

    fsm::retval StateCrossGate::genPath()
    {
        path = std::make_shared<sisl::Path>();
        std::vector<Eigen::Vector3d> points = {
            { 0.0, 0.0, 0.0 },
            { 1.0, 2.0, 0.0 },
            { 3.0, 0.0, 0.0 }
        };

        int degree = 2;
        std::vector<double> weights = { 1.0, 1.0, 1.0 };
        std::vector<double> knots = { 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 };
        std::vector<double> coefficients; // Let it be auto-computed

        std::shared_ptr<sisl::GenericCurve> curve = std::make_shared<sisl::GenericCurve>(
            degree, knots, points, weights, coefficients);

        path->AddCurveBack(curve);

        auto sampledPoints = path->Sampling(10);
        std::cout << "Sampled path points:\n";
        for (const auto& pt : *sampledPoints) {
            std::cout << pt.transpose() << "\n";
        }

        return fsm::ok;
    }

}
}
