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
        Eigen::Vector3d distanceVector(0,2,0);
        ctb::LatLong pos;
        ctb::LocalNED2LatLong(distanceVector, ctb::LatLong(0, 0), pos, alt);
        try {
            GateBuoy gb1(ctb::LatLong(0, 0));
            GateBuoy gb2(pos);
            Gate gt(gb1, gb2);
            gate = std::make_shared<Gate>(gt);
        } catch (const std::exception& e) {
            std::cerr << "Error in detected gate: " << e.what() << std::endl;
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

    // //     void temp(){
    //     std::string type = jvalues["type"].asString();
    //     double angle = jvalues["params"]["angle"].asDouble();
    //     double size_1_Path = jvalues["params"]["size_1"].asDouble();
    //     double size_2_Path = jvalues["params"]["size_2"].asDouble();
    //     sisl::Path::Direction direction = static_cast<sisl::Path::Direction>(jvalues["params"]["direction"].asInt());
    //     std::string polypathType = jvalues["params"]["polypath_type"].asString();

    //     std::shared_ptr<sisl::Path> newPath;
    //     bool pathCreated{true};

    //     futils::Timer executionTime;
    //     executionTime.Start();
    //     try {
    //         if (polypathType == "Serpentine") {
    //             newPath = sisl::PathFactory::NewSerpentine(angle, direction, size_1_Path, polyVerticesUTM);
    //         } else if (polypathType == "RaceTrack") {
    //             newPath = sisl::PathFactory::NewRaceTrack(angle, direction, size_1_Path, size_2_Path, polyVerticesUTM);
    //         } else if (polypathType == "Hippodrome") {

    //             Eigen::Vector3d baricenter;
    //             for(int i = 0; i < 3; i++) {
    //                 double dim_sum{0};
    //                 for(size_t j = 0; j < (polyVerticesUTM.size() - 1); j++) {
    //                     dim_sum += polyVerticesUTM.at(j)[i];
    //                 }
    //                 baricenter[i] = dim_sum/(polyVerticesUTM.size() - 1);
    //             }
    //             //std::cout << "Lap 1:" << executionTime.Elapsed() << std::endl;
    //             newPath = sisl::PathFactory::NewHippodrome(-angle, direction, size_1_Path, size_2_Path, baricenter);
    //         } else {
    //             std::cout << "[CommandWrapper::createPathFromPolygon] polypathType '" << polypathType << "' not recognized." << std::endl;
    //             pathCreated = false;
    //         }
    //     }
    //     catch(std::runtime_error const& exception) {
    //         std::cout << "Received exception from --> " << exception.what() << std::endl;
    //     }

    //     //std::cout << "Lap 2:" << executionTime.Elapsed() << std::endl;
    //     QVector<double> pathVectorGeo;
    //     if (pathCreated) {
    //         std::cout << *newPath << std::endl;
    //         // Sampling the curve and converting it back to lat-long for visualization on map
    //         //double samplingInterval = 1.0; // meters
    //         //int numSamples = (int)round(newPath->Length()/samplingInterval);
    //         double numSamples = 512;
    //         auto sampledPath = newPath->Sampling(numSamples);

    //         //std::cout << "Lap 3:" << executionTime.Elapsed() << std::endl;
    //         for(size_t i=0; i < sampledPath->size(); i++){
    //             ctb::LatLong pathPointGeo;
    //             ctb::LocalUTM2LatLong(sampledPath->at(i), centroid, pathPointGeo, altitude);
    //             pathVectorGeo << pathPointGeo.latitude << pathPointGeo.longitude;
    //         }
    //     }
    //         bool CommandWrapper::sendPath(const QString &pathJsonData)
    // {
    //     auto serviceReq = std::make_shared<ulisse_msgs::srv::ControlCommand::Request>();
    //     serviceReq->command_type = ulisse::commands::ID::pathfollow;

    //     // DEBUG PRINT
    //     //QJsonDocument doc = QJsonDocument::fromJson(pathJsonData.toUtf8());
    //     //QString formattedJsonString = doc.toJson(QJsonDocument::Indented);
    //     //std::cout << formattedJsonString.toStdString() << std::endl;

    //     Json::Reader reader;
    //     Json::Value jObj;

    //     reader.parse(pathJsonData.toStdString(), jObj);

    //     serviceReq->path_cmd.path.id = jObj["name"].asString();
    //     serviceReq->path_cmd.path.type = jObj["type"].asString();

    //     serviceReq->path_cmd.path.polypath_type =  jObj["params"]["polypath_type"].asString();
    //     serviceReq->path_cmd.path.angle = jObj["params"]["angle"].asDouble();
    //     serviceReq->path_cmd.path.size_1 = jObj["params"]["size_1"].asDouble();
    //     serviceReq->path_cmd.path.size_2 = jObj["params"]["size_2"].asDouble();
    //     serviceReq->path_cmd.path.direction = jObj["params"]["direction"].asBool();

    //     serviceReq->path_cmd.path.centroid.latitude = jObj["centroid"]["latitude"].asDouble();
    //     serviceReq->path_cmd.path.centroid.longitude = jObj["centroid"]["longitude"].asDouble();

    //     serviceReq->path_cmd.path.coordinates.resize(jObj["coordinates"].size());
    //     unsigned int i = 0;
    //     try {
    //         for (const Json::Value &coord : jObj["coordinates"]) {

    //             serviceReq->path_cmd.path.coordinates.at(i).latitude = coord["latitude"].asDouble();
    //             serviceReq->path_cmd.path.coordinates.at(i).longitude = coord["longitude"].asDouble();

    //             i++;
    //         }
    //     } catch (Json::Exception& e) {
    //         // output exception information
    //         std::cout << "Error parsing QML Jason" << e.what() << std::endl;
    //     }

    //     return SendCommandRequest(serviceReq);
    // }
    //     }

}
}
