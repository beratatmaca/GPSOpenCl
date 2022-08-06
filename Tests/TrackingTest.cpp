#include "../Source/GPSOpenClSettings.h"
#include "../Source/GPSOpenClTracking.h"

#include "GPSOpenClCommon.h"
#include "TestUtils.h"

namespace GPSOpenClTest
{
class TrackingTest : public testing::Test
{
  public:
    GPSOpenCl::Settings m_settings;
    GPSOpenCl::Tracking *m_tracking;

  protected:
    void SetUp() override
    {
        m_settings.captureSettings();
        m_tracking = new GPSOpenCl::Tracking(m_settings.configuration);
    }

    void TearDown() override
    {
        delete m_tracking;
        m_tracking = NULL;
    }
};

TEST_F(TrackingTest, Test1)
{
    GPSOpenCl::ComplexFloatVector outputVec;
    GPSOpenCl::ComplexFloatVector expectedOutputVec;
    GPSOpenCl::ComplexFloatVector inputSignal;

    TestUtils::readFromFileComplex("../../Tests/Scripts/Tracking/ncoMultiplication.txt", &expectedOutputVec);
    TestUtils::readFromFileComplex("../../Tests/Scripts/inputSignal.txt", &inputSignal);

    int codeLength = m_settings.configuration.rawDataSettings.numberOfSamplesPerCode;

    auto start = inputSignal.begin();
    auto end = inputSignal.begin() + codeLength;
    GPSOpenCl::ComplexFloatVector inputSignalCodeLenghtClipped(start, end);

    EXPECT_EQ(1, 1);
}
} // namespace GPSOpenClTest
