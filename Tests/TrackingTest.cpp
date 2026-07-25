#include "GPSOpenClSettings.h"
#include "GPSOpenClTracking.h"

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

TEST_F(TrackingTest, NcoMultiplication)
{
    GPSOpenCl::ComplexFloatVector outputVec;
    GPSOpenCl::ComplexFloatVector expectedOutputVec;

    TestUtils::readFromFileComplex("../../Tests/Scripts/Tracking/ncoMultiplication.txt", &expectedOutputVec);
    if (expectedOutputVec.empty())
    {
        TestUtils::readFromFileComplex("Scripts/Tracking/ncoMultiplication.txt", &expectedOutputVec);
    }

    int codeLength = m_settings.configuration.rawDataSettings.numberOfSamplesPerCode;
    GPSOpenCl::ComplexFloatVector unitInput(codeLength, std::complex<float>(1.0f, 0.0f));

    m_tracking->ncoMultiplicate(unitInput, 3100.0f, &outputVec);

    if (!expectedOutputVec.empty())
    {
        TestUtils::compareComplexResults(outputVec, expectedOutputVec, 0.01);
    }
    else
    {
        EXPECT_EQ(outputVec.size(), unitInput.size());
    }
}

TEST_F(TrackingTest, TrackingLoop)
{
    GPSOpenCl::ComplexFloatVector outputVec;
    GPSOpenCl::ComplexFloatVector inputSignal;

    TestUtils::readFromFileComplex("../../Tests/Scripts/inputSignal.txt", &inputSignal);
    if (inputSignal.empty())
    {
        TestUtils::readFromFileComplex("Scripts/inputSignal.txt", &inputSignal);
    }

    int codeLength = m_settings.configuration.rawDataSettings.numberOfSamplesPerCode;
    if (inputSignal.size() < static_cast<size_t>(codeLength))
    {
        inputSignal.resize(codeLength, std::complex<float>(1.0f, 0.0f));
    }

    auto start = inputSignal.begin();
    auto end = inputSignal.begin() + codeLength;
    GPSOpenCl::ComplexFloatVector inputSignalClipped(start, end);

    m_tracking->doWork(inputSignalClipped, 1, &outputVec);

    EXPECT_EQ(outputVec.size(), 1u);
}
} // namespace GPSOpenClTest
