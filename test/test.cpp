/**
* license(c) 2018 RoboSense(Suteng Innovation Technology Co., Ltd.)
* All rights reserved.
* \author glmhit@126.com   2018-1-30
* \ingroup test
*/
#include <gtest/gtest.h>

TEST(Add, BaseFeature) {
  EXPECT_EQ(-3, -3);
  EXPECT_GT(4, -6); // 故意的
  EXPECT_GT(2, 6);
}

int main(int argc, char **argv) {
  std::cout << "Running main() from gtest_main.cc\n";

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
