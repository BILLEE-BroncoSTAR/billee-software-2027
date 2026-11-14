#include <gtest/gtest.h>
#include "comms/Gps.hpp"

TEST(GpsMsgPayloads, MatchWireLayout)
{
  EXPECT_EQ(sizeof(posPayload), 28U);
  EXPECT_EQ(sizeof(covPayload), 64U);
}
