#include "../Source/GPSOpenClCode.h"
#include "../Source/GPSOpenClCommon.h"
#include "../Source/GPSOpenClSettings.h"

#include "TestUtils.h"

namespace GPSOpenClTest
{
class CaCodeTest : public testing::Test
{
  public:
    GPSOpenCl::Compute m_gpuCompute;

  protected:
    void SetUp() override
    {
    }

    void TearDown() override
    {
    }
};

TEST_F(CaCodeTest, Test1)
{
    GPSOpenCl::Settings m_settings;
    m_settings.configuration.rawDataSettings.samplingFrequency = 2048000.0;
    m_settings.configuration.rawDataSettings.numberOfSamplesPerCode = 2048;

    GPSOpenCl::Code m_code = GPSOpenCl::Code(m_settings.configuration);
    m_code.createLookupTable(&m_gpuCompute);
    GPSOpenCl::FloatVector testInputVec;

    TestUtils::readFromFileReal("../../Tests/Scripts/CACode/CaCodeTestCase1.txt", &testInputVec);

    int offset = 0;

    for (int i = 1; i <= GPSOpenCl::GPS_CA_SV_COUNT; i++)
    {
        auto start = testInputVec.begin() + offset;
        auto end = testInputVec.begin() + offset + m_settings.configuration.rawDataSettings.numberOfSamplesPerCode;
        GPSOpenCl::FloatVector vector(start, end);

        TestUtils::compareRealResults(m_code.m_upsampledCaCode[i - 1], vector, 0.0000001);

        offset += m_settings.configuration.rawDataSettings.numberOfSamplesPerCode;
    }
}

TEST_F(CaCodeTest, Test2)
{
    GPSOpenCl::Settings m_settings;
    m_settings.configuration.rawDataSettings.samplingFrequency = 4096000.0;
    m_settings.configuration.rawDataSettings.numberOfSamplesPerCode = 4096;

    GPSOpenCl::Code m_code = GPSOpenCl::Code(m_settings.configuration);
    m_code.createLookupTable(&m_gpuCompute);
    GPSOpenCl::FloatVector testInputVec;

    TestUtils::readFromFileReal("../../Tests/Scripts/CACode/CaCodeTestCase2.txt", &testInputVec);

    int offset = 0;

    for (int i = 1; i <= GPSOpenCl::GPS_CA_SV_COUNT; i++)
    {
        auto start = testInputVec.begin() + offset;
        auto end = testInputVec.begin() + offset + m_settings.configuration.rawDataSettings.numberOfSamplesPerCode;
        GPSOpenCl::FloatVector vector(start, end);

        TestUtils::compareRealResults(m_code.m_upsampledCaCode[i - 1], vector, 0.0000001);

        offset += m_settings.configuration.rawDataSettings.numberOfSamplesPerCode;
    }
}

TEST_F(CaCodeTest, Test3)
{
    GPSOpenCl::Settings m_settings;
    m_settings.configuration.rawDataSettings.samplingFrequency = 8192000.0;
    m_settings.configuration.rawDataSettings.numberOfSamplesPerCode = 8192;

    GPSOpenCl::Code m_code = GPSOpenCl::Code(m_settings.configuration);
    m_code.createLookupTable(&m_gpuCompute);
    GPSOpenCl::FloatVector testInputVec;

    TestUtils::readFromFileReal("../../Tests/Scripts/CACode/CaCodeTestCase3.txt", &testInputVec);

    int offset = 0;

    for (int i = 1; i <= GPSOpenCl::GPS_CA_SV_COUNT; i++)
    {
        auto start = testInputVec.begin() + offset;
        auto end = testInputVec.begin() + offset + m_settings.configuration.rawDataSettings.numberOfSamplesPerCode;
        GPSOpenCl::FloatVector vector(start, end);

        TestUtils::compareRealResults(m_code.m_upsampledCaCode[i - 1], vector, 0.0000001);

        offset += m_settings.configuration.rawDataSettings.numberOfSamplesPerCode;
    }
}

TEST_F(CaCodeTest, Test4)
{
    GPSOpenCl::Settings m_settings;
    m_settings.configuration.rawDataSettings.samplingFrequency = 16384000.0;
    m_settings.configuration.rawDataSettings.numberOfSamplesPerCode = 16384;

    GPSOpenCl::Code m_code = GPSOpenCl::Code(m_settings.configuration);
    m_code.createLookupTable(&m_gpuCompute);
    GPSOpenCl::FloatVector testInputVec;

    TestUtils::readFromFileReal("../../Tests/Scripts/CACode/CaCodeTestCase4.txt", &testInputVec);

    int offset = 0;

    for (int i = 1; i <= GPSOpenCl::GPS_CA_SV_COUNT; i++)
    {
        auto start = testInputVec.begin() + offset;
        auto end = testInputVec.begin() + offset + m_settings.configuration.rawDataSettings.numberOfSamplesPerCode;
        GPSOpenCl::FloatVector vector(start, end);

        TestUtils::compareRealResults(m_code.m_upsampledCaCode[i - 1], vector, 0.0000001);

        offset += m_settings.configuration.rawDataSettings.numberOfSamplesPerCode;
    }
}
} // namespace GPSOpenClTest
