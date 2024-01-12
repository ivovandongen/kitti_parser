#include "timestamp.hpp"

#include <sstream>
#include <chrono>
#include <iomanip>

long toUnixTimestamp(const std::string &datetimeString) {
    std::istringstream ss(datetimeString);

    std::tm tmStruct = {};
    ss >> std::get_time(&tmStruct, "%Y-%m-%d %H:%M:%S");
    auto timePoint = std::chrono::system_clock::from_time_t(std::mktime(&tmStruct) - timezone);

    double fractionalSeconds;
    ss >> fractionalSeconds;
    auto milliseconds = static_cast<int64_t>(fractionalSeconds * 1e3);

    return std::chrono::duration_cast<std::chrono::milliseconds>(
            (timePoint + std::chrono::milliseconds(milliseconds)).time_since_epoch()).count();
}
