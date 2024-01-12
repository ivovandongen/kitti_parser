#include <kitti_parser/util/timestamp.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>


TEST(Timestamp, parseTimestamp) {
    std::string timestamp{"2011-09-26 13:12:05.825305088"};
    long unixTimestampL = toUnixTimestamp(timestamp);
    ASSERT_EQ(unixTimestampL, 1317042725825);
}

TEST(Timestamp, parseTimestampFile) {
    std::ifstream testFile{std::filesystem::path(FIXTURES_DIR) / "timestamps.txt"};
    std::ifstream expectedFile{std::filesystem::path(FIXTURES_DIR) / "timestamps_expected.txt"};

    std::string inputS;
    std::string expectedS;
    while (std::getline(testFile, inputS) && std::getline(expectedFile, expectedS)) {
        long input = toUnixTimestamp(inputS);
        long expected = std::stol(expectedS);
        EXPECT_EQ(input, expected);
    }
}