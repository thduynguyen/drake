#include "gtest/gtest.h"
#include "godotvis/message.hpp"

TEST(message_test, content) {
  EXPECT_EQ(get_message(), "Hello world!");
}
