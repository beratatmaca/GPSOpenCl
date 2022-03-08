#include <gtest/gtest.h>
#include <iostream>
#include "Settings/Settings.h"

namespace GPSOpenClTest
{
    class SettingsTest : public ::testing::Test
    {
    protected:
        SettingsTest()
        {
            GPSOpenCl::Settings setting;
            // You can do set-up work for each test here.
        }

        virtual ~SettingsTest()
        {
            // You can do clean-up work that doesn't throw exceptions here.
        }

        // If the constructor and destructor are not enough for setting up
        // and cleaning up each test, you can define the following methods:

        virtual void SetUp()
        {
            // Code here will be called immediately after the constructor (right
            // before each test).
        }

        virtual void TearDown()
        {
            // Code here will be called immediately after each test (right
            // before the destructor).
        }

        // Objects declared here can be used by all tests in the test case for SettingsTest.
    };
    TEST_F(SettingsTest, TestSettings)
    {
        GPSOpenCl::Settings setting;
        setting.readIniFile("../../Source/default.ini");
        std::string resultStr = "";
        setting.getSetting("fileName", resultStr);
        EXPECT_EQ(resultStr, "gpssim.bin");
        setting.getSetting("samplingFrequency", resultStr);
        EXPECT_EQ(resultStr, "16384000");
        setting.getSetting("dataType", resultStr);
        EXPECT_EQ(resultStr, "int8");
    }
}

