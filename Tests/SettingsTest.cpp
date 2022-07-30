#include "../Source/GPSOpenClSettings.h"

#include "TestUtils.h"

namespace GPSOpenClTest
{
class SettingsTest : public testing::Test
{
  public:
    GPSOpenCl::Settings m_settings;

  protected:
    void SetUp() override
    {
    }

    void TearDown() override
    {
    }
};

TEST_F(SettingsTest, ReadIniFile)
{
    m_settings.captureSettings();

    EXPECT_EQ(m_settings.configuration.rawDataSettings.dataSource, std::string("capture.dat"));
    EXPECT_EQ(m_settings.configuration.rawDataSettings.samplingFrequency, 4096000);
    EXPECT_EQ(m_settings.configuration.rawDataSettings.numberOfSamplesPerCode, 4096);
    EXPECT_EQ(m_settings.configuration.acquisitionSettings.acquisitionDopplerMinimum, -4000);
    EXPECT_EQ(m_settings.configuration.acquisitionSettings.acquisitionDopplerMaximum, 4000);
    EXPECT_EQ(m_settings.configuration.acquisitionSettings.acquisitionDopplerSearchRange, 500);
}
} // namespace GPSOpenClTest
