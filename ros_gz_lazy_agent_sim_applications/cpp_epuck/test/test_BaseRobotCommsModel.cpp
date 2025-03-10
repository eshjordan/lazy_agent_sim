#include "mock_NetworkFactory.hpp"
#include <cpp_epuck/RobotCommsModel.hpp>
#include <gtest/gtest.h>

// // File is just an entrypoint to run the unit tests from
// int main(int argc, char **argv)
// {
//     testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }

class TestBaseRobotCommsModel : public ::testing::Test
{
};

TEST_F(TestBaseRobotCommsModel, TestGetSeq) {}

TEST_F(TestBaseRobotCommsModel, TestKnownIdsBegin) {}

TEST_F(TestBaseRobotCommsModel, TestKnownIdsEnd) {}

TEST_F(TestBaseRobotCommsModel, TestKnownIdsSize) {}

TEST_F(TestBaseRobotCommsModel, TestInsertKnownIds) {}

TEST_F(TestBaseRobotCommsModel, TestCreateKnowledgePacket) {}

TEST_F(TestBaseRobotCommsModel, TestSetCentroid) {}

TEST_F(TestBaseRobotCommsModel, TestSetBoundary) {}

TEST_F(TestBaseRobotCommsModel, TestGetCentroid) {}

TEST_F(TestBaseRobotCommsModel, TestGetBoundary) {}
