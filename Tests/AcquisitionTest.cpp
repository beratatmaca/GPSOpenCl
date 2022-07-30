#include "../Source/GPSOpenClAcquisition.h"
#include "../Source/GPSOpenClChannel.h"
#include "../Source/GPSOpenClGPUCompute.h"
#include "../Source/GPSOpenClSettings.h"

#include "GPSOpenClCommon.h"
#include "TestUtils.h"

#define DOPPLER_TOLERANCE 0.01
#define DETECTION_TOLERANCE 0.01f

namespace GPSOpenClTest
{
class AcquisitionTest : public testing::Test
{
  public:
    GPSOpenCl::Settings m_settings;
    GPSOpenCl::Acquisition *m_acquisition;
    GPSOpenCl::Compute m_gpu;
    GPSOpenCl::Channel m_channels[GPSOpenCl::GPS_CA_SV_COUNT];

  protected:
    void SetUp() override
    {
        m_settings.captureSettings();
        m_acquisition = new GPSOpenCl::Acquisition(m_settings.configuration);

        for (int i = 0; i < GPSOpenCl::GPS_CA_SV_COUNT; i++)
        {
            m_channels[i].m_svId = i + 1;
        }
    }

    void TearDown() override
    {
        delete m_acquisition;
        m_acquisition = NULL;
    }
};

TEST_F(AcquisitionTest, AcquisitionMetrics)
{
    GPSOpenCl::FloatVector outputVec;
    GPSOpenCl::FloatVector expectedOutputVec;
    GPSOpenCl::ComplexFloatVector inputSignal;
    int calculatedPeakIndex;
    float calculatedPeakValue;
    float calculatedPeakFrequency;
    float calculatedMeanValue;
    float calculatedCn0;
    float calculatedPeakRatio;

    GPSOpenCl::Code m_code = GPSOpenCl::Code(m_settings.configuration);
    m_code.createLookupTable(&m_gpu);

    TestUtils::readFromFileReal("../../Tests/Scripts/Acquisition/AcquisitionMetrics.txt", &expectedOutputVec);
    TestUtils::readFromFileComplex("../../Tests/Scripts/inputSignal.txt", &inputSignal);

    int codeLength = m_settings.configuration.rawDataSettings.numberOfSamplesPerCode;

    auto start = inputSignal.begin();
    auto end = inputSignal.begin() + codeLength;
    GPSOpenCl::ComplexFloatVector inputSignalCodeLenghtClipped(start, end);

    int counter = 0;
    for (int i = 1; i <= GPSOpenCl::GPS_CA_SV_COUNT; i++)
    {
        auto timeBegin = TestUtils::startElapsedTimeMeasurement();

        m_acquisition->correlate(inputSignalCodeLenghtClipped, &m_gpu, &m_code, &m_channels[i - 1]);

        std::string desc = "Correlator for PRN " + std::to_string(i);
        TestUtils::measureElapsedTime(desc, timeBegin);

        m_channels[i - 1].getAcquisitionResults(&calculatedPeakIndex, &calculatedPeakValue, &calculatedPeakFrequency,
                                                &calculatedMeanValue, &calculatedCn0, &calculatedPeakRatio);

        outputVec.clear();

        int svID = static_cast<int>(expectedOutputVec.at(counter));
        counter++;
        EXPECT_EQ(i, svID);

        float peakValue = expectedOutputVec.at(counter);
        counter++;
        TestUtils::compareRealResults(calculatedPeakValue, peakValue, 0.01);

        float peakIndex = expectedOutputVec.at(counter) - 1;
        counter++;
        EXPECT_NEAR(calculatedPeakIndex, peakIndex, 0.001);

        float peakFrequency = expectedOutputVec.at(counter);
        counter++;
        EXPECT_NEAR(calculatedPeakFrequency, peakFrequency, 0.001);

        float meanValue = expectedOutputVec.at(counter);
        counter++;
        TestUtils::compareRealResults(calculatedMeanValue, meanValue, 0.01);

        float cnO = expectedOutputVec.at(counter);
        counter++;
        TestUtils::compareRealResults(calculatedCn0, cnO, 0.01);
    }
}
} // namespace GPSOpenClTest
