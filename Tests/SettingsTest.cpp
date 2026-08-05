#include "../Source/GPSOpenClSettings.h"

#include "TestUtils.h"

#include <cstdio>
#include <fstream>

namespace GPSOpenClTest
{
class SettingsTest : public testing::Test
{
  public:
    GPSOpenCl::Settings m_settings;

  protected:
    void SetUp() override {}

    void TearDown() override {}
};

TEST_F(SettingsTest, ReadIniFile)
{
    m_settings.captureSettings();

    EXPECT_EQ(std::string(m_settings.configuration.sourceInput.fifoPath), std::string("capture.dat"));
    EXPECT_EQ(m_settings.configuration.acquisitionInput.samplingFrequencyHz, 4'096'000);
    EXPECT_EQ(m_settings.configuration.acquisitionInput.numberOfSamplesPerCode, 4096);
    EXPECT_EQ(m_settings.configuration.acquisitionInput.acquisitionDopplerMinimum, -4000);
    EXPECT_EQ(m_settings.configuration.acquisitionInput.acquisitionDopplerMaximum, 4000);
    EXPECT_EQ(m_settings.configuration.acquisitionInput.acquisitionDopplerSearchRange, 500);
}

class SettingsValidationTest : public testing::Test
{
  protected:
    void writeConf(const std::string &body)
    {
        std::ofstream file("DefaultConf.ini");
        file << body;
    }

    void TearDown() override { std::remove("DefaultConf.ini"); }
};

TEST_F(SettingsValidationTest, RejectsZeroDopplerSearchRange)
{
    writeConf("AcquisitionDopplerSearchRange = 0\n");

    GPSOpenCl::Settings settings;
    settings.captureSettings();

    EXPECT_EQ(settings.configuration.acquisitionInput.acquisitionDopplerSearchRange, 500);
    EXPECT_EQ(settings.configuration.acquisitionInput.acquisitionDopplerSearchRange, 500);
}

TEST_F(SettingsValidationTest, RejectsInvertedDopplerWindow)
{
    writeConf("AcquisitionMinimumDoppler = 4000\nAcquisitionMaximumDoppler = -4000\n");

    GPSOpenCl::Settings settings;
    settings.captureSettings();

    EXPECT_EQ(settings.configuration.acquisitionInput.acquisitionDopplerMinimum, -4000);
    EXPECT_EQ(settings.configuration.acquisitionInput.acquisitionDopplerMaximum, 4000);
}

TEST_F(SettingsValidationTest, RejectsNonFiniteSamplingFrequency)
{
    writeConf("SamplingFrequency = nan\n");

    GPSOpenCl::Settings settings;
    settings.captureSettings();

    EXPECT_EQ(settings.configuration.acquisitionInput.samplingFrequencyHz, 4'096'000);
    EXPECT_EQ(settings.configuration.acquisitionInput.numberOfSamplesPerCode, 4096);
    EXPECT_EQ(settings.configuration.acquisitionInput.numberOfSamplesPerCode, 4096);
}

TEST_F(SettingsValidationTest, RejectsNegativeSamplingFrequency)
{
    writeConf("SamplingFrequency = -4096000\n");

    GPSOpenCl::Settings settings;
    settings.captureSettings();

    EXPECT_EQ(settings.configuration.acquisitionInput.samplingFrequencyHz, 4'096'000);
    EXPECT_EQ(settings.configuration.acquisitionInput.numberOfSamplesPerCode, 4096);
    EXPECT_GT(settings.configuration.trackingInput.numberOfSamplesPerCode, 0);
}

TEST_F(SettingsValidationTest, CommentAndSectionLinesAreIgnored)
{
    writeConf("# AcquisitionDopplerSearchRange = 9\n"
              "; SamplingFrequency = 1\n"
              "[Acquisition]\n"
              "AcquisitionDopplerSearchRange = 250\n");

    GPSOpenCl::Settings settings;
    settings.captureSettings();

    EXPECT_EQ(settings.configuration.acquisitionInput.acquisitionDopplerSearchRange, 250);
    EXPECT_EQ(settings.configuration.acquisitionInput.numberOfSamplesPerCode, 4096);
}
}
