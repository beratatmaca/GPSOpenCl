#include <gtest/gtest.h>

/**
 * @brief 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */

int main(int argc, char **argv)
{
   ::testing::GTEST_FLAG(output) = "xml:TestResult.xml";
   ::testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}