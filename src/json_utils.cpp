#include <json_utils/json_utils.hpp>
#include <boost/algorithm/string/replace.hpp>

namespace ctljsn
{

  void CheckName(std::string name, std::vector<std::string> names)
  {
    if (std::find(names.begin(), names.end(), name) == names.end())
    { // name not found in names
      std::string listOfOkNames("");
      for (auto n : names)
        listOfOkNames.append(n + " ");
      throw std::invalid_argument(name + " is invalid! Should be one among: " + listOfOkNames);
    }
  }

  void CheckRange(const double x, const double xMin, const double xMax, std::string quantityName)
  {
    if ((x > xMax) || (x < xMin))
    {
      throw std::out_of_range(quantityName + " outside range! Should be between " + std::to_string(xMin) + " and " + std::to_string(xMax) +
                              " but the value is " + std::to_string(x));
    }
  }

  jsoncons::json MinMax::ToJson() const
  {
    jsoncons::json result;
    result["minimum"] = xmin;
    result["maximum"] = xmax;
    return result;
  }

  jsoncons::json CATLIdentifier::ToJson() const
  {
    jsoncons::json result;
    result["name"] = name;
    result["value"] = value;
    return result;
  }

  namespace task
  {
    jsoncons::json task::TaskTypeVN::ToJson() const
    {
      jsoncons::json result;
      result["name"] = name;
      result["value"] = value;
      return result;
    }

    std::shared_ptr<TaskPerformance> GetTaskPerformanceFromJson(jsoncons::json jsonData, TaskType taskType)
    {
      switch (taskType)
      {
      case TaskType::TSKTP_STANDBY:
      {
        return std::make_shared<TaskPerformanceStandby>(jsonData);
      }
      case TaskType::TSKTP_SURVEY:
      {
        return std::make_shared<TaskPerformanceSurvey>(jsonData);
      }
      case TaskType::TSKTP_SYNCHRONISATION:
      {
        return std::make_shared<TaskPerformanceSynchronisation>(jsonData);
      }
      case TaskType::TSKTP_PREPARATION:
      {
        return std::make_shared<TaskPerformancePreparation>(jsonData);
      }
      case TaskType::TSKTP_U_HOLD:
      {
        return std::make_shared<TaskPerformanceBasic>(jsonData);
      }
      case TaskType::TSKTP_U_MOVE_TO_LATLONG:
      {
        return std::make_shared<TaskPerformanceBasic>(jsonData);
      }
      case TaskType::TSKTP_U_SURGE_HEADING:
      {
        return std::make_shared<TaskPerformanceBasic>(jsonData);
      }
      case TaskType::TSKTP_U_SURGE_YAWRATE:
      {
        return std::make_shared<TaskPerformanceBasic>(jsonData);
      }
      case TaskType::TSKTP_U_PATH_FOLLOW:
      {
        return std::make_shared<TaskPerformanceBasic>(jsonData);
      }
      default:
      {
        throw std::logic_error("[GetTaskPerformanceFromJson] Not yet implemented for task type = " + std::string(ToString(taskType)));
      }
      }
    }

    std::shared_ptr<TaskConstraints> GetTaskConstraintsFromJson(jsoncons::json jsonData, TaskType taskType)
    {
      switch (taskType)
      {
      case TaskType::TSKTP_STANDBY:
      {
        return std::make_shared<TaskConstraintsStandBy>(jsonData);
      }
      case TaskType::TSKTP_SURVEY:
      {
        return std::make_shared<TaskConstraintsSurvey>(jsonData);
      }
      case TaskType::TSKTP_SYNCHRONISATION:
      {
        return std::make_shared<TaskConstraintsSynchronisation>(jsonData);
      }
      case TaskType::TSKTP_PREPARATION:
      {
        return std::make_shared<TaskConstraintsPreparation>(jsonData);
      }
      case TaskType::TSKTP_U_HOLD:
      {
        return std::make_shared<TaskConstraintsBasic>(jsonData);
      }
      case TaskType::TSKTP_U_MOVE_TO_LATLONG:
      {
        return std::make_shared<TaskConstraintsBasic>(jsonData);
      }
      case TaskType::TSKTP_U_SURGE_HEADING:
      {
        return std::make_shared<TaskConstraintsBasic>(jsonData);
      }
      case TaskType::TSKTP_U_SURGE_YAWRATE:
      {
        return std::make_shared<TaskConstraintsBasic>(jsonData);
      }
      case TaskType::TSKTP_U_PATH_FOLLOW:
      {
        return std::make_shared<TaskConstraintsBasic>(jsonData);
      }
      default:
      {
        throw std::logic_error("[GetTaskConstraintsFromJson] Not yet implemented for task type = " + std::string(ToString(taskType)));
      }
      }
    }
  }

  CATLIdentifier::CATLIdentifier(const jsoncons::json &jsonData)
  {
    name = jsonData["name"].as<std::string>();
    value = jsonData["value"].as<int>();
  };

  jsoncons::json CATLIdentifierStrStr::ToJson() const
  {
    jsoncons::json result;
    result["name"] = name;
    result["value"] = value;
    return result;
  }

  jsoncons::json CATLHeader::ToJson() const
  {
    jsoncons::json header;
    header["message_type"] = message_type;
    header["source"] = source;
    header["time_sent"] = time_sent->ToJson();
    return header;
  }

  jsoncons::json Status::ToJson() const
  {

    jsoncons::json body;
    body["id"] = vehicleId;
    body["status"] = ToString(nodeStatus);
    //  std::cerr << tc::yellow << "body[status] = " << body["status"] << tc::none << std::endl;

    auto capabilitiesJson = jsoncons::json::make_array<1>(0);
    for (auto c : capabilities)
      capabilitiesJson.push_back(c.ToJson());
    body["capabilities"] = capabilitiesJson;

    auto owned_tasksJson = jsoncons::json::make_array<1>(0);
    for (auto p : owned_tasks)
      owned_tasksJson.push_back(p.ToJson());
    body["owned_tasks"] = owned_tasksJson;
    body["location"] = location.ToJson();

    jsoncons::json status;
    status["header"] = header.ToJson();
    status["body"] = body;

    return status;
  }

  namespace nd
  {
    jsoncons::json SpatialParams::ToJson() const
    {
      jsoncons::json result;
      result["depth"] = depth.ToJson();
      result["altitude"] = altitude.ToJson();
      result["z"] = z.ToJson();
      return result;
    }

    jsoncons::json OtherParams::ToJson() const
    {
      jsoncons::json result;
      result["endurance"] = endurance->ToJson();
      return result;
    }

    jsoncons::json EnvironmentalParams::ToJson() const
    {
      jsoncons::json result;
      result["current"] = current.ToJson();
      result["sea_state"] = sea_state.ToJson();
      result["wind_speed"] = wind_speed.ToJson();
      result["temperature"] = temperature.ToJson();
      result["salinity"] = salinity.ToJson();
      return result;
    }

    jsoncons::json OperatingEnvelope::ToJson() const
    {
      jsoncons::json result;
      result["environmental_params"] = environmental_params.ToJson();
      result["spatial_params"] = spatial_params.ToJson();
      result["other_params"] = other_params.ToJson();
      return result;
    }

    jsoncons::json TimeWindow::ToJson() const
    {
      jsoncons::json result;
      result["minimum"] = minimum->ToJson();
      result["maximum"] = maximum->ToJson();
      return result;
    }

    jsoncons::json Initialization::ToJson() const
    {
      jsoncons::json result;
      result["time_window"] = time_window.ToJson();
      result["launch_region"] = launch_region.ToJson();
      return result;
    }

    jsoncons::json AddNode::ToJson() const
    {
      jsoncons::json result;
      result["header"] = header.ToJson();
      result["body"]["identifier"] = identifier.ToJson();
      result["body"]["capabilities"] = GetJSonArrayOfJsonable(capabilities);
      result["body"]["config_options"] = GetJSonArrayOfJsonable(config_options);
      result["body"]["operating_envelope"] = operating_envelope.ToJson();
      result["body"]["initialization"] = initialization.ToJson();
      result["body"]["assigned_exclusion_zones"] = GetJSonArrayOfPureObjects(assigned_exclusion_zones);
      result["body"]["assigned_inclusion_zones"] = GetJSonArrayOfPureObjects(assigned_inclusion_zones);
      return result;
    }

  }

  namespace time
  {

    DirectDuration::DirectDuration(double durationInSeconds)
    {
      // Separate the duration into its integer and fractional parts
      int totalSeconds = static_cast<int>(durationInSeconds);
      double fractionalSeconds = durationInSeconds - totalSeconds;

      // Calculate hours, minutes, and seconds from totalSeconds
      int hours = totalSeconds / 3600;
      int minutes = (totalSeconds % 3600) / 60;
      int seconds = totalSeconds % 60;

      // Calculate milliseconds from fractionalSeconds
      int milliseconds = static_cast<int>(fractionalSeconds * 1000);

      // Create the ISO 8601 duration string
      std::ostringstream oss;
      oss << "PT";
      if (hours > 0)
      {
        oss << hours << "H";
      }
      if (minutes > 0)
      {
        oss << minutes << "M";
      }
      if (seconds > 0 || (hours == 0 && minutes == 0))
      {
        if (milliseconds > 0)
        {
          oss << std::fixed << std::setprecision(3) << seconds + fractionalSeconds << "S";
        }
        else
        {
          oss << seconds << "S";
        }
      }

      data = oss.str();
    }

    DirectTime::DirectTime(time_t unixTimestamp)
    {
      // Convert unix timestamp to tm structure
      std::tm *gmt = std::gmtime(&unixTimestamp);

      // Create a string stream to format the date and time
      std::ostringstream oss;
      oss << std::put_time(gmt, "%Y-%m-%dT%H:%M:%S") << "+00:00";

      data = oss.str();
    }

    jsoncons::json EpochTimePeriod::ToJson() const
    {
      jsoncons::json result;
      result["identifier"] = identifier.ToJson();
      result["epoch"] = epoch.ToJson();
      result["time_step"] = time_step.ToJson();
      result["duration"] = duration.ToJson();
      return result;
    }

    jsoncons::json DirectDuration::ToJson() const
    {
      jsoncons::json result;
      result["data"] = data;
      return json_query(result, "$.data");
    }

    jsoncons::json DirectTime::ToJson() const
    {
      jsoncons::json result;
      result["data"] = data;
      return json_query(result, "$.data");
    }

    jsoncons::json IndexedDuration::ToJson() const
    {
      jsoncons::json result;
      result["epoch_time_period"] = epoch_time_period.ToJson();
      result["index"] = index;
      return result;
    }

    jsoncons::json IndexedTime::ToJson() const
    {
      jsoncons::json result;
      result["epoch_time_period"] = epoch_time_period.ToJson();
      result["index"] = index;
      return result;
    }

    std::shared_ptr<DeltaT> CreateDeltaTFromJson(const jsoncons::json &jsonData)
    {

      if (jsonData.contains("epoch_time_period") && jsonData.contains("index"))
      {
        // It's an IndexedDuration
        int index = jsonData["index"].as<int>();
        EpochTimePeriod epoch_time_period(jsonData["epoch_time_period"]);
        return std::make_shared<IndexedDuration>(epoch_time_period, index);
      }
      else
      {
        // It's a DirectDuration
        auto data = jsonData[0].as<std::string>();
        return std::make_shared<DirectDuration>(data);
      }
    }

    std::shared_ptr<AbsoluteTime> CreateAbsoluteTimeFromJson(const jsoncons::json &jsonData)
    {

      /*
      if (jsonData.contains("epoch_time_period") && jsonData.contains("index")) {
          // It's an IndexedDuration
          int index = jsonData["index"].as<int>();
          EpochTimePeriod epoch_time_period(jsonData["epoch_time_period"]);
          return std::make_shared<IndexedTime>(epoch_time_period, index);
      } else {
          // It's a DirectDuration
          std::string data = jsonData[0].as<std::string>();
          return std::make_shared<DirectTime>(data);
      }
      */
      if (jsonData.contains("epoch_time_period") && jsonData.contains("index"))
      {
        // It's an IndexedDuration
        int index = jsonData["index"].as<int>();
        EpochTimePeriod epoch_time_period(jsonData["epoch_time_period"]);
        return std::make_shared<IndexedTime>(epoch_time_period, index);
      }
      else
      {
        // It's a DirectDuration
        if (jsonData["body"]["mission_data"].contains("end_time"))
        {
          // Se il campo "end_time" è un array, estrai la prima data
          std::string data = jsonData["body"]["mission_data"]["end_time"][0].as<std::string>();
          return std::make_shared<DirectTime>(data);
        }
        else
        {
          std::cerr << "Errore: campo 'end_time' mancante o malformato." << std::endl;
          return nullptr;
        }
      }
    }
  }

  namespace task
  {

    jsoncons::json TaskAdmin::ToJson() const
    {

      jsoncons::json body;
      body["action"] = ToString(action);
      body["node"] = node.ToJson();
      body["type"]["name"] = ToString(taskType);
      body["identifier"] = taskID.ToJson();
      if (descriptorIsSet_)
        body["description"] = taskDescriptor.ToJson();

      jsoncons::json taskUpdate;
      taskUpdate["header"] = header.ToJson();
      taskUpdate["body"] = body;
      return taskUpdate;
    }

    jsoncons::json TaskStatus::ToJson() const
    {
      jsoncons::json taskStatusMsg;
      CheckRange(percentComplete, 0, 1, "task percentage of completion");
      taskStatusMsg["state"]["name"] = ToString(taskState);
      taskStatusMsg["state"]["percent_complete"] = percentComplete;
      auto timeRemainingJson = time_remaining->ToJson();
      taskStatusMsg["time_remaining"] = timeRemainingJson;
      return taskStatusMsg;
    }

    jsoncons::json TaskConstraints::ToJson() const
    {
      jsoncons::json result;
      result["activity"] = ToString(activityType);
      for (auto pair : dict)
      {
        result[pair.first] = pair.second;
      }
      return result;
    }

    jsoncons::json TaskConstraintsBasic::ToJson() const
    {
      auto result = TaskConstraints::ToJson();
      if (isPosSet)
        result["target"] = p.ToJson();
      for (auto k : taskConstraintKeys)
      {
        auto searchRes = dict.find(k);
        if (searchRes != dict.end())
          result[k] = searchRes->second;
      }
      return result;
    }

    jsoncons::json TaskPerformance::ToJson() const
    {
      jsoncons::json result;
      for (auto pair : dict)
      {
        result[pair.first] = pair.second;
      }
      return result;
    };

    jsoncons::json TaskPerformanceBasic::ToJson() const
    {
      auto result = TaskPerformance::ToJson();
      result["timeout"] = timeout->ToJson();
      return result;
    }

    jsoncons::json TaskPerformancePreparation::ToJson() const
    {
      auto result = TaskPerformance::ToJson();
      result["start_time"] = start_time->ToJson();
      result["end_time"] = end_time->ToJson();
      return result;
    }

    jsoncons::json TaskPerformanceStandby::ToJson() const
    {
      auto result = TaskPerformance::ToJson();
      result["start_time"] = start_time->ToJson();
      result["end_time"] = end_time->ToJson();
      return result;
    }

    jsoncons::json TaskPerformanceSynchronisation::ToJson() const
    {
      auto result = TaskPerformance::ToJson();
      result["acknowledge"] = acknowledge;
      return result;
    }

    jsoncons::json TaskPerformanceSurvey::ToJson() const
    {
      auto result = TaskPerformance::ToJson();
      return result;
    }

    jsoncons::json TaskConstraintsSurvey::ToJson() const
    {
      auto result = TaskConstraints::ToJson();
      result["region"] = region;
      auto deadlineJson = deadline->ToJson();
      result["deadline"] = deadline->ToJson();
      return result;
    }

    jsoncons::json TaskConstraintsPreparation::ToJson() const
    {
      auto result = TaskConstraints::ToJson();
      result["region"] = region.ToJson();
      return result;
    }

    jsoncons::json TaskConstraintsStandBy::ToJson() const
    {
      auto result = TaskConstraints::ToJson();
      result["region"] = region;
      return result;
    }

    jsoncons::json TaskConstraintsSynchronisation::ToJson() const
    {
      auto result = TaskConstraints::ToJson();
      result["region"] = region.ToJson();
      result["destination_node_id"] = nodeIdentifier.ToJson();
      return result;
    }

    jsoncons::json TaskIdAndStatus::ToJson() const
    {
      auto result = taskStatus.ToJson();
      result["identifier"] = taskId.ToJson();
      return result;
    }

    jsoncons::json TaskDescriptor::ToJson() const
    {
      jsoncons::json result;
      if (taskConstraints != nullptr)
        result["constraints"] = taskConstraints->ToJson();
      if (taskPerformance != nullptr)
        result["performance_envelope"] = taskPerformance->ToJson();
      return result;
    }

  }

  namespace geographic
  {

    jsoncons::json Position::ToJson() const
    {
      jsoncons::json result;
      if (usesData)
        result = data;
      else
        result["reference"] = query;
      return result;
    }

    jsoncons::json GenerateLatLongPosition(const double latitude, const double longitude)
    {
      CheckRange(latitude, LATITUDE_MIN, LATITUDE_MAX, "Latitude");
      CheckRange(longitude, LONGITUDE_MIN, LONGITUDE_MAX, "Latitude");

      jsoncons::json latlongMsg;
      latlongMsg["latitude"] = latitude;
      latlongMsg["longitude"] = longitude;
      return latlongMsg;
    }

    jsoncons::json GenerateVerticalPosition(const std::map<std::string, double> positions)
    {
      jsoncons::json verticalPosMsg;
      for (auto p : positions)
      {
        CheckName(p.first, verticalPositionNames);
        verticalPosMsg[p.first] = p.second;
      }
      return verticalPosMsg;
    }

    jsoncons::json GenerateASVPosition(const double latitude, const double longitude, const double depth)
    {
      jsoncons::json posMsg;
      posMsg["horizontal"] = geographic::GenerateLatLongPosition(latitude, longitude);
      posMsg["vertical"] = geographic::GenerateVerticalPosition({{"depth", depth}});
      return posMsg;
    }

    jsoncons::json GenerateENUPosition(const double E, const double N, bool useHeight, const double U)
    {
      jsoncons::json enuMsg;
      enuMsg["rast"] = E;
      enuMsg["north"] = N;
      if (useHeight)
        enuMsg["up"] = U;
      return enuMsg;
    }

    jsoncons::json GenerateMilitaryGrid(const std::string grid_zone_designator, const std::string square_identifier, const std::string easting_northing)
    {
      jsoncons::json militaryGridMsg;
      militaryGridMsg["grid_zone_designator"] = grid_zone_designator;
      militaryGridMsg["square_identifier"] = square_identifier;
      militaryGridMsg["easting_northing"] = easting_northing;
      return militaryGridMsg;
    }

    jsoncons::json GenerateDefinedRegion(std::vector<jsoncons::json> boundariesItems)
    {
      jsoncons::json boundaries, regionMsg;
      boundaries = jsoncons::json::make_array<1>(0);
      for (auto p : boundariesItems)
        boundaries.push_back(p);
      regionMsg["boundaries"] = boundaries;
      return regionMsg;
    }

    /*double* CreateLatLongPositionFromJson(const jsoncons::json& jsonData) {
      static double lat_long[2];
      if (jsonData["body"]["resources"]["spatial_primitives"]["objects"][0]["position"].contains("horizontal")) {
          lat_long[0] = jsonData["body"]["resources"]["spatial_primitives"]["objects"][0]["position"]["horizontal"]["latitude"].as<double>();
          lat_long[1] = jsonData["body"]["resources"]["spatial_primitives"]["objects"][0]["position"]["horizontal"]["longitude"].as<double>();
          return lat_long;
      }
      else {
          std::cerr << "Errore: json non valido o malformato" << std::endl;
          return nullptr;
      }
    }*/

    double *CreateLatLongPositionFromJson(const jsoncons::json &jsonData)
    {
      static double lat_long[2];

      try
      {

        // formato DYNAMIC_UPDATE (lat/long in body -> operations[0] -> value -> position -> horizontal)
        if (jsonData["body"].contains("operations") &&
            !jsonData["body"]["operations"].empty() &&
            jsonData["body"]["operations"][0].contains("value"))
        {

          const auto &horizontal = jsonData["body"]["operations"][0]["value"]["position"]["horizontal"];
          lat_long[0] = horizontal["latitude"].as<double>();
          lat_long[1] = horizontal["longitude"].as<double>();
          return lat_long;
        }

        std::cerr << "Errore: formato JSON non riconosciuto" << std::endl;
        return nullptr;
      }
      catch (const std::exception &e)
      {
        std::cerr << "Errore nell'estrazione di lat/long: " << e.what() << std::endl;
        return nullptr;
      }
    }

    std::string handle_status(const jsoncons::json &status_msg)
    {
      auto name = status_msg["body"]["identifier"]["name"].as<std::string>();
      auto status = status_msg["body"]["status"].as<std::string>();

      if (status == "AVAILABLE")
      {
        return name + " is IDLE";
      }
      else if (status == "ON_TASK")
      {
        const auto &tasks = status_msg["body"]["owned_tasks"];

        if (tasks.empty())
        {
          return name + " is ON_TASK but no task details found.";
        }

        std::ostringstream out;
        out << name << " is ON_TASK:\n";

        for (const auto &task : tasks.array_range())
        {
          std::string task_name = task["identifier"]["name"].as<std::string>();
          std::string state = task["state"].as<std::string>();
          double percent = task["percent_complete"].as<double>();
          std::string remaining = task["time_remaining"].as<std::string>();

          out << "- Task: " << task_name << "\n"
              << "  State: " << state << "\n"
              << "  Completion: " << percent << "%\n"
              << "  Time remaining: " << remaining << "\n";
        }

        return out.str();
      }
      else
      {
        return name + " has unknown status: " + status;
      }
    }

  }

  namespace security
  {
    jsoncons::json Markings::ToJson() const
    {
      jsoncons::json markingsJson;
      markingsJson["classification"] = ToString(classification);
      return markingsJson;
    }
  }

  jsoncons::json Contact::ToJson() const
  {
    jsoncons::json result;
    result["identifier"] = identifier;
    result["position"] = position.ToJson();
    return result;
  }

  jsoncons::json ASWContact::ToJson() const
  {
    return Contact::ToJson();
  }

  jsoncons::json MCMContact::ToJson() const
  {
    auto result = Contact::ToJson();
    result["positional_uncertainty"] = positional_uncertainty;
    result["state"] = ToString(mcmContactState);
    result["classification"]["type"] = classificationType;
    result["classification"]["confidence"] = confidence;
    return result;
  }

  bool ResolveRef(jsoncons::json &t, const std::shared_ptr<wm::WorldModel> &wm, jsoncons::json &result, std::string text)
  {
    if (t.contains("reference"))
    {
      auto refStr = t["reference"].to_string();
      auto query = refStr.substr(3, refStr.size() - 6);
      std::cerr << "[ResolveRef] query = " << refStr << std::endl;
      std::cerr << "[ResolveRef] cleaned query = " << query << std::endl;
      jsoncons::json queryResult;
      if (wm != nullptr)
        queryResult = json_query(wm->ToJson()["body"].as<jsoncons::json>(), query);
      if ((wm == nullptr) || queryResult.empty())
      {
        std::cerr << tc::redL << "[ResolveRef] No result found. Not proceeding to " << text << std::endl;
        return false;
      }
      std::cerr << "[ResolveRef] query result = " << queryResult << std::endl;
      result = queryResult.at(0);
      return true;
    }
    result = t;
    return true;
  }

  jsoncons::json DynamicUpdate::ToJson() const
  {
    auto result = jsoncons::json();
    result["header"] = header.ToJson();
    result["body"]["target"] = target;
    auto opsJ = jsoncons::json::make_array<1>(0);
    for (auto p : ops)
      opsJ.push_back(p);
    result["body"]["operations"] = opsJ;
    return result;
  }

  std::vector<std::string> splitString(const std::string &str, char delimiter)
  {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(str);
    while (std::getline(tokenStream, token, delimiter))
    {
      tokens.push_back(token);
    }
    return tokens;
  }

  std::string LocationToJsonPointerString(jsoncons::jsonpath::json_location loc)
  {
    auto s = to_string(loc);
    boost::replace_all(s, "['", "/");
    boost::replace_all(s, "']", "");
    boost::replace_all(s, "$", "");
    return s;
  }

  bool EvalDynamicUpdate2(DynamicUpdate &du, std::shared_ptr<JSonable> wm, jsoncons::json &result, std::string text)
  {
    result = wm->ToJson();
    std::cerr << "wm before = " << result << std::endl;
    auto opsVec = du.ops;
    std::cerr << "[DynamicUpdate] opsVec.size = " << opsVec.size() << std::endl;
    for (auto operation : opsVec)
    {
      auto action = operation.at("action").as_string();
      auto path = operation.at("key").at("reference");
      // auto item = jsoncons::jsonpointer::get(wm->ToJson(),path.to_string()); // "foo"
      std::cerr << "[DynamicUpdate] Case " << action << std::endl;
      std::cerr << "[DynamicUpdate] Path is " << path.to_string() << std::endl;
      std::error_code ec;
      auto pathStr = path.to_string();
      pathStr = pathStr.substr(1, pathStr.size() - 2);
      auto expr = jsonpath::make_expression<json>(pathStr);
      std::vector<jsonpath::json_location> locations = expr.select_paths(result, jsonpath::result_options::sort_descending);
      auto exprFather = jsonpath::make_expression<json>(pathStr + "^");
      std::vector<jsonpath::json_location> locationsFather = exprFather.select_paths(result, jsonpath::result_options::sort_descending);

      std::cerr << "[DynamicUpdate] OK " << std::endl;
      if (action == "DELETE")
      {
        if (locations.size() > 0)
          jsonpath::remove(result, locations[0]);
      }
      else if (action == "PUT")
      {
        auto value = operation.at("value");
        if (locations.size() > 0)
        {
          jsonpath::remove(result, locations[0]);
          auto locStr = LocationToJsonPointerString(locations[0]);
          auto locStrFather = LocationToJsonPointerString(locationsFather[0]);
          auto oldData = jsonpointer::get(result, locStrFather, ec);
          if (oldData.is_array())
          {
            oldData.emplace_back(value);
          }
          else
          {
            for (const auto &member : value.object_range())
            {
              oldData[member.key()] = member.value();
              std::cout << member.key() << " => " << member.value().as<std::string>() << std::endl;
            }
          }
          jsonpointer::replace(result, locStrFather, oldData, ec);
        }
      }
      else if ((action == "APPEND"))
      {
        std::cerr << "[DynamicUpdate] APPEND " << action << std::endl;
        auto value = operation.at("value");
        auto locStr = LocationToJsonPointerString(locations[0]);
        std::cerr << "locStr = " << locStr << std::endl;
        auto arr = jsonpointer::get(result, locStr, ec);
        if (arr.is_array())
        {
          arr.emplace_back(value);
        }
        else
        {
          throw std::logic_error("[EvalDynamicUpdate2] APPEND operation not implemented for non-arrays!");
        }
        jsonpointer::replace(result, locStr, arr, ec);
      }
      else if (action == "UPDATE")
      {
        throw std::runtime_error("[DynamicUpdate] Case PUT not yet implemented!");
      }
      else if (action == "GET")
      {
        throw std::runtime_error("[DynamicUpdate] Case GET not yet implemented!");
      }
      else
      {
        throw std::runtime_error("[DynamicUpdate] Case " + action + " not yet implemented!");
      }

      std::cerr << "[DynamicUpdate] OK 2" << std::endl;
      if (ec)
        std::cerr << tc::redL << "[DynamicUpdate] Err: " << ec.message() << tc::none << std::endl;
      else
        std::cerr << tc::greenL << "[DynamicUpdate] OK!" << tc::none << std::endl;
      std::cerr << "wm after = " << result << std::endl;
      if (result["body"]["resources"]["spatial_primitives"].contains("positions"))
        std::cerr << tc::yellow << "wm pos after = " << result["body"]["resources"]["spatial_primitives"]["positions"]
                  << tc::none << std::endl;
      else
        std::cerr << tc::yellow << "positions not found" << tc::none << std::endl;
    }
    return true;
  }

  bool EvalDynamicUpdate(DynamicUpdate &du, std::shared_ptr<JSonable> wm, jsoncons::json &result, std::string text)
  {
    auto wmJson = wm->ToJson();
    std::cerr << "wm before = " << wmJson << std::endl;
    auto opsVec = du.ops;
    std::cerr << "[DynamicUpdate] opsVec.size = " << opsVec.size() << std::endl;
    for (auto operation : opsVec)
    {
      auto action = operation.at("action").as_string();
      auto path = operation.at("key").at("reference");
      // auto item = jsoncons::jsonpointer::get(wm->ToJson(),path.to_string()); // "foo"
      std::cerr << "[DynamicUpdate] Case " << action << std::endl;
      std::cerr << "[DynamicUpdate] Path is " << path.to_string() << std::endl;
      std::error_code ec;
      auto pathStr = path.to_string();
      pathStr = pathStr.substr(1, pathStr.size() - 2);
      if (action == "DELETE")
      {
        jsoncons::jsonpointer::remove(wmJson, pathStr, ec);
      }
      /*else if (action == "UPDATE") {
        auto value = operation.at("value");
        //auto oldData = jsonpointer::get(wmJson, pathStr, ec);
        //for (const auto& member : value.object_range()) {
        //    oldData[member.key()] = member.value();
        //    std::cout << member.key() << " => " << member.value().as<std::string>() << std::endl;
        //}
        //jsonpointer::replace(wmJson, pathStr, oldData, ec);
        jsonpointer::replace(wmJson, pathStr, value, ec);
      }*/
      else if ((action == "APPEND") || (action == "UPDATE"))
      {
        if (pathStr.find("[") == std::string::npos)
        {
          auto value = operation.at("value");
          auto oldData = jsonpointer::get(wmJson, pathStr, ec);
          for (const auto &member : value.object_range())
          {
            oldData[member.key()] = member.value();
            std::cout << member.key() << " => " << member.value().as<std::string>() << std::endl;
          }
          jsonpointer::replace(wmJson, pathStr, oldData, ec);
        }
        else
        {
        }
      }
      else if (action == "PUT")
      {
        throw std::runtime_error("[DynamicUpdate] Case PUT not yet implemented!");
      }
      else if (action == "GET")
      {
        throw std::runtime_error("[DynamicUpdate] Case GET not yet implemented!");
      }
      else
      {
        throw std::runtime_error("[DynamicUpdate] Case " + action + " not yet implemented!");
      }

      if (ec)
        std::cerr << tc::redL << "[DynamicUpdate] Err: " << ec.message() << tc::none << std::endl;
      else
        std::cerr << tc::greenL << "[DynamicUpdate] OK!" << tc::none << std::endl;
      std::cerr << "wm after = " << wmJson << std::endl;
    }
    return true;
  }

  jsoncons::json GenerateDynamicUpdateItem(std::string action, std::string ref, jsoncons::json value)
  {
    jsoncons::json op;
    op["action"] = action;
    op["key"]["reference"] = ref;
    if (action.find("DELETE") == std::string::npos)
      op["value"] = value;
    return op;
  }

  namespace wm
  {
    jsoncons::json Resource::ToJson() const
    {
      jsoncons::json resourceJson;
      if (markingsIsSet)
        resourceJson["markings"] = markings.ToJson();
      resourceJson["spatial_primitives"] = spatial_primitives.ToJson();
      resourceJson["labels"] = labels.ToJson();
      return resourceJson;
    }

    jsoncons::json MissionData::ToJson() const
    {
      jsoncons::json result;
      auto inclusion_zonesArr = jsoncons::json::make_array<1>(0);
      for (auto p : inclusion_zones)
        inclusion_zonesArr.push_back(p.ToJson());
      result["inclusion_zones"] = inclusion_zonesArr;

      auto exclusion_zonesArr = jsoncons::json::make_array<1>(0);
      for (auto p : exclusion_zones)
        exclusion_zonesArr.push_back(p.ToJson());
      result["exclusion_zones"] = exclusion_zonesArr;

      auto contactsArr = jsoncons::json::make_array<1>(0);
      for (auto p : contacts)
        contactsArr.push_back(p->ToJson());
      result["contacts"] = contactsArr;

      result["end_time"] = end_time->ToJson();

      if (isDataProductsSet)
      {
        result["data_products"] = dataProducts.ToJson();
      }

      return result;
    }

    jsoncons::json Assignment::ToJson() const
    {
      jsoncons::json result;
      return result;
    }

    jsoncons::json Terrain::ToJson() const
    {
      jsoncons::json result;
      result["terrain_type"] = terrainType;
      result["boundary"]["region"] = region.ToJson();
      return result;
    }

    jsoncons::json Currents::ToJson() const
    {
      jsoncons::json result;
      result["speed"] = speed;
      result["direction"] = direction;
      result["boundary"]["region"] = region.ToJson();
      return result;
    }

    jsoncons::json Bathymetry::ToJson() const
    {
      jsoncons::json result;
      result["location"] = location.ToJson();
      result["depth"] = depth;
      return result;
    }

    jsoncons::json Maps::ToJson() const
    {
      jsoncons::json result;
      auto occupancyArr = jsoncons::json::make_array<1>(0);
      for (auto p : occupancy)
        occupancyArr.push_back(p.ToJson());
      result["occupancy"]["assignments"] = occupancyArr;

      auto terrainArr = jsoncons::json::make_array<1>(0);
      for (auto p : terrain)
        terrainArr.push_back(p.ToJson());
      result["terrain"] = terrainArr;

      auto currentsArr = jsoncons::json::make_array<1>(0);
      for (auto p : currents)
        currentsArr.push_back(p.ToJson());
      result["currents"] = currentsArr;

      auto bathymetryArr = jsoncons::json::make_array<1>(0);
      for (auto p : bathymetry)
        bathymetryArr.push_back(p.ToJson());
      result["bathymetry"] = bathymetryArr;

      return result;
    }

    jsoncons::json LabelledSpace::ToJson() const
    {
      jsoncons::json regJson;
      regJson["identifier"] = identifier.ToJson();
      if (spaceType == SpaceType::POS)
        regJson["position"] = region.ToJson();
      else if (spaceType == SpaceType::REG)
        regJson["region"] = region.ToJson();
      else if (spaceType == SpaceType::OBJ)
        regJson["object"] = region.ToJson();
      return regJson;
    }

    jsoncons::json SpatialPrimitives::ToJson() const
    {
      jsoncons::json result;
      result["positions"] = GetJSonArrayOfJsonable(positions);
      result["regions"] = GetJSonArrayOfJsonable(regions);
      result["objects"] = GetJSonArrayOfJsonable(objects);
      return result;
    }

    jsoncons::json FlexEnum::ToJson() const
    {
      jsoncons::json result;
      result["name"] = name;
      result["dsec"] = desc;
      result["list"] = GetJSonArrayOfJsonable(enumerations);
      return result;
    }

    jsoncons::json WorldModel::ToJson() const
    {
      jsoncons::json worldModelJSon;
      worldModelJSon["header"] = header.ToJson();
      worldModelJSon["body"]["resources"] = resources.ToJson();
      worldModelJSon["body"]["mission_data"] = missionData.ToJson();
      worldModelJSon["body"]["maps"] = maps.ToJson();
      worldModelJSon["body"]["enumerations"] = GetJSonArrayOfJsonable<FlexEnum>(enumerations);
      return worldModelJSon;
    }

    jsoncons::json DataProducts::ToJson() const
    {
      jsoncons::json dataProductsJson;
      dataProductsJson["identifier"] = identifier.ToJson();

      auto tasksArr = jsoncons::json::make_array<1>(0);
      for (auto tsk : relevant_tasks)
        tasksArr.push_back(tsk.ToJson());
      dataProductsJson["relevant_tasks"] = tasksArr;

      auto productsArr = jsoncons::json::make_array<1>(0);
      for (auto s : products)
        productsArr.push_back(s);
      dataProductsJson["products"] = productsArr;
      return dataProductsJson;
    }

    jsoncons::json Labels::ToJson() const
    {
      jsoncons::json labelsJson;
      auto arr = jsoncons::json::make_array<1>(0);
      for (auto p : bottom_types)
        arr.push_back(p);
      labelsJson["bottom_types"] = arr;
      return labelsJson;
    }

    void WorldModel::Merge(const WorldModel &other)
    {
      MergeVectors<std::string>(resources.labels.bottom_types, other.resources.labels.bottom_types);
      MergeVectors<wm::LabelledSpace>(resources.spatial_primitives.objects, other.resources.spatial_primitives.objects);
      MergeVectors<wm::LabelledSpace>(resources.spatial_primitives.positions, other.resources.spatial_primitives.positions);
      MergeVectors<wm::LabelledSpace>(resources.spatial_primitives.regions, other.resources.spatial_primitives.regions);
      MergeVectorsOfPtr<Contact>(missionData.contacts, other.missionData.contacts);
      // missionData.dataProducts; // TODO implement
      MergeVectors<wm::LabelledSpace>(missionData.exclusion_zones, other.missionData.exclusion_zones);
      MergeVectors<wm::LabelledSpace>(missionData.inclusion_zones, other.missionData.inclusion_zones);
      MergeVectors<wm::Bathymetry>(maps.bathymetry, other.maps.bathymetry);
      MergeVectors<wm::Currents>(maps.currents, other.maps.currents);
      MergeVectors<wm::Assignment>(maps.occupancy, other.maps.occupancy);
      MergeVectors<wm::Terrain>(maps.terrain, other.maps.terrain);
      MergeVectors<wm::FlexEnum>(enumerations, other.enumerations);
    }
  }

  jsoncons::json GeneralSourceOfInformation::ToJson() const
  {
    auto result = CapabilityAuthority::ToJson();
    result["group"] = group;
    result["organization"] = organization;
    return result;
  }

  jsoncons::json CapabilityDescriptor::ToJson() const
  {
    jsoncons::json result;
    result["type"] = type.ToJson();
    if (authorities.size() > 0)
    {
      auto authoritiesJson = jsoncons::json::make_array<1>(0);
      for (auto a : authorities)
        authoritiesJson.push_back(a->ToJson());
      result["Authorities"] = authoritiesJson;
    }
    return result;
  }

  namespace chat
  {
    jsoncons::json Chat::ToJson() const
    {
      jsoncons::json result;
      result["header"] = header.ToJson();
      result["body"]["identifier"] = identifier;
      result["body"]["text"] = text;
      result["body"]["author"] = author;
      return result;
    }
  }

  void DebugStatus(const jsoncons::json &status)
  {
    std::cout << tc::yellow << "status = " << status << tc::none << std::endl;

    auto header = json_query(status, "$.header");
    std::cout << tc::yellow << "header = " << header << tc::none << std::endl;

    auto time_sent = json_query(status, "$.header.time_sent");
    std::cout << tc::yellow << "time_sent = " << time_sent << tc::none << tc::none << std::endl;

    auto capabilities = json_query(status, "$.body.capabilities");
    std::cout << tc::yellow << "capabilities = " << capabilities << tc::none << tc::none << std::endl;

    auto owned_tasks = json_query(status, "$.body.owned_tasks");
    std::cout << tc::yellow << "owned_tasks = " << owned_tasks << tc::none << tc::none << std::endl;

    auto location = json_query(status, "$.body.location");
    std::cout << tc::yellow << "location = " << location << tc::none << tc::none << std::endl;
  }

  void Test1()
  {

    // Construct a book object
    jsoncons::json book1;

    book1["category"] = "Fiction";
    book1["title"] = "A Wild Sheep Chase: A Novel";
    book1["author"] = "Haruki Murakami";
    book1["date"] = "2002-04-09";
    book1["price"] = 9.01;
    book1["isbn"] = "037571894X";

    // Construct another using the member function insert_or_assign
    jsoncons::json book2;

    book2.insert_or_assign("category", "History");
    book2.insert_or_assign("title", "Charlie Wilson's War");
    book2.insert_or_assign("author", "George Crile");
    book2.insert_or_assign("date", "2007-11-06");
    book2.insert_or_assign("price", 10.50);
    book2.insert_or_assign("isbn", "0802143415");

    // Use insert_or_assign again, but more efficiently
    jsoncons::json book3;

    // Reserve memory, to avoid reallocations
    book3.reserve(6);

    // Insert in name alphabetical order
    // Give insert_or_assign a hint where to insert the next member
    auto hint = book3.insert_or_assign(book3.object_range().begin(), "author", "Haruki Murakami");
    hint = book3.insert_or_assign(hint, "category", "Fiction");
    hint = book3.insert_or_assign(hint, "date", "2006-01-03");
    hint = book3.insert_or_assign(hint, "isbn", "1400079276");
    hint = book3.insert_or_assign(hint, "price", 13.45);
    hint = book3.insert_or_assign(hint, "title", "Kafka on the Shore");

    // Construct a fourth from a string
    jsoncons::json book4 = jsoncons::json::parse(R"(
    {
        "category" : "Fiction",
        "title" : "Pulp",
        "author" : "Charles Bukowski",
        "date" : "2004-07-08",
        "price" : 22.48,
        "isbn" : "1852272007"  
    }
    )");

    // Construct a booklist array
    jsoncons::json booklist(json_array_arg);

    // For efficiency, reserve memory, to avoid reallocations
    booklist.reserve(4);

    // For efficency, tell jsoncons to move the contents
    // of the four book objects into the array
    booklist.push_back(std::move(book1));
    booklist.push_back(std::move(book2));

    // Add the third one to the front
    auto where = booklist.insert(booklist.array_range().begin(), std::move(book3));

    // Add the last one immediately after
    booklist.insert(where + 1, std::move(book4));

    // See what's left of book1, 2, 3 and 4 (expect nulls)
    std::cout << book1 << "," << book2 << "," << book3 << "," << book4 << std::endl;

    // Loop through the booklist elements using a range-based for loop
    for (auto book : booklist.array_range())
    {
      std::cout << book["title"].as<std::string>()
                << ","
                << book["price"].as<double>() << std::endl;
    }

    // The second book
    jsoncons::json &book = booklist[1];

    // Loop through the book members using a range-based for loop
    for (auto member : book.object_range())
    {
      std::cout << member.key()
                << ","
                << member.value() << std::endl;
    }

    auto it = book.find("author");
    if (it != book.object_range().end())
    {
      // member "author" found
    }

    if (book.contains("author"))
    {
      // book has member "author"
    }

    std::string s = book.get_value_or<std::string>("author", "author unknown");
    // Returns author if found, otherwise "author unknown"

    try
    {
      book["ratings"].as<std::string>();
    }
    catch (const std::out_of_range &)
    {
      // member "ratings" not found
    }

    // Add ratings
    book["ratings"]["*****"] = 4;
    book["ratings"]["*"] = 1;

    // Delete one-star ratings
    book["ratings"].erase("*");

    // Serialize the booklist to a file
    std::ofstream os("./store.json");
    os << pretty_print(booklist);
  }

  void Test2()
  {
    // Deserialize the booklist
    std::ifstream is("./store.json");
    jsoncons::json booklist;
    is >> booklist;
    std::cerr << booklist.as_string() << std::endl;
    jsoncons::json result;
    // Use a JSONPath expression to find

    // (1) The authors of books that cost more than $0
    result = json_query(booklist, "$[*].title");
    std::cout << "(0) " << result << std::endl;

    // (1) The authors of books that cost less than $12
    result = json_query(booklist, "$[?(@.price < 12)].title");
    std::cout << "(1) " << result << std::endl;

    // (2) The number of books
    result = json_query(booklist, "$.length");
    std::cout << "(2) " << result << std::endl;

    // (3) The third book
    result = json_query(booklist, "$[2]");
    std::cout << "(3) " << std::endl
              << pretty_print(result) << std::endl;

    // (4) The authors of books that were published in 2004
    result = json_query(booklist, "$[?(@.date =~ /2007.*?/)].title");
    std::cout << "(4) " << result << std::endl;

    // (5) The titles of all books that have ratings
    result = json_query(booklist, "$[?(@.ratings)].title");
    std::cout << "(5) " << result << std::endl;
  }
}
