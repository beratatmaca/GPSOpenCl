#include "Acquisition/GPSOpenClCaCodeGenerator.hpp"
#include "Common/GPSOpenClCommon.hpp"
#include "Common/GPSOpenClSettings.hpp"

#include "TestUtils.hpp"

namespace GPSOpenClTest
{
class CaCodeTest : public testing::Test
{
  public:
    GPSOpenCl::SpectrumEngine m_gpuCompute;

  protected:
    void SetUp() override {}

    void TearDown() override {}
};

TEST_F(CaCodeTest, Test1)
{
    GPSOpenCl::Settings m_settings;
    m_settings.configuration.acquisitionInput.samplingFrequencyHz = 2048000.0;
    m_settings.configuration.acquisitionInput.numberOfSamplesPerCode = 2048;

    GPSOpenCl::CaCodeGenerator m_code = GPSOpenCl::CaCodeGenerator(m_settings.configuration);
    m_code.createLookupTable(&m_gpuCompute);
    GPSOpenCl::FloatVector testInputVec;

    TestUtils::readFromFileReal("../../Tests/Scripts/CACode/CaCodeTestCase1.txt", &testInputVec);

    int offset = 0;

    for (int i = 1; i <= GPSOpenCl::GPS_CA_SV_COUNT; i++)
    {
        auto start = testInputVec.begin() + offset;
        auto end = testInputVec.begin() + offset + m_settings.configuration.acquisitionInput.numberOfSamplesPerCode;
        GPSOpenCl::FloatVector vector(start, end);

        TestUtils::compareRealResults(m_code.upsampledCaCode[i - 1], vector, 0.0000001);

        offset += m_settings.configuration.acquisitionInput.numberOfSamplesPerCode;
    }
}

TEST_F(CaCodeTest, Test2)
{
    GPSOpenCl::Settings m_settings;
    m_settings.configuration.acquisitionInput.samplingFrequencyHz = 4096000.0;
    m_settings.configuration.acquisitionInput.numberOfSamplesPerCode = 4096;

    GPSOpenCl::CaCodeGenerator m_code = GPSOpenCl::CaCodeGenerator(m_settings.configuration);
    m_code.createLookupTable(&m_gpuCompute);
    GPSOpenCl::FloatVector testInputVec;

    TestUtils::readFromFileReal("../../Tests/Scripts/CACode/CaCodeTestCase2.txt", &testInputVec);

    int offset = 0;

    for (int i = 1; i <= GPSOpenCl::GPS_CA_SV_COUNT; i++)
    {
        auto start = testInputVec.begin() + offset;
        auto end = testInputVec.begin() + offset + m_settings.configuration.acquisitionInput.numberOfSamplesPerCode;
        GPSOpenCl::FloatVector vector(start, end);

        TestUtils::compareRealResults(m_code.upsampledCaCode[i - 1], vector, 0.0000001);

        offset += m_settings.configuration.acquisitionInput.numberOfSamplesPerCode;
    }
}

TEST_F(CaCodeTest, Test3)
{
    GPSOpenCl::Settings m_settings;
    m_settings.configuration.acquisitionInput.samplingFrequencyHz = 8192000.0;
    m_settings.configuration.acquisitionInput.numberOfSamplesPerCode = 8192;

    GPSOpenCl::CaCodeGenerator m_code = GPSOpenCl::CaCodeGenerator(m_settings.configuration);
    m_code.createLookupTable(&m_gpuCompute);
    GPSOpenCl::FloatVector testInputVec;

    TestUtils::readFromFileReal("../../Tests/Scripts/CACode/CaCodeTestCase3.txt", &testInputVec);

    int offset = 0;

    for (int i = 1; i <= GPSOpenCl::GPS_CA_SV_COUNT; i++)
    {
        auto start = testInputVec.begin() + offset;
        auto end = testInputVec.begin() + offset + m_settings.configuration.acquisitionInput.numberOfSamplesPerCode;
        GPSOpenCl::FloatVector vector(start, end);

        TestUtils::compareRealResults(m_code.upsampledCaCode[i - 1], vector, 0.0000001);

        offset += m_settings.configuration.acquisitionInput.numberOfSamplesPerCode;
    }
}

TEST_F(CaCodeTest, Test4)
{
    GPSOpenCl::Settings m_settings;
    m_settings.configuration.acquisitionInput.samplingFrequencyHz = 16384000.0;
    m_settings.configuration.acquisitionInput.numberOfSamplesPerCode = 16'384;

    GPSOpenCl::CaCodeGenerator m_code = GPSOpenCl::CaCodeGenerator(m_settings.configuration);
    m_code.createLookupTable(&m_gpuCompute);
    GPSOpenCl::FloatVector testInputVec;

    TestUtils::readFromFileReal("../../Tests/Scripts/CACode/CaCodeTestCase4.txt", &testInputVec);

    int offset = 0;

    for (int i = 1; i <= GPSOpenCl::GPS_CA_SV_COUNT; i++)
    {
        auto start = testInputVec.begin() + offset;
        auto end = testInputVec.begin() + offset + m_settings.configuration.acquisitionInput.numberOfSamplesPerCode;
        GPSOpenCl::FloatVector vector(start, end);

        TestUtils::compareRealResults(m_code.upsampledCaCode[i - 1], vector, 0.0000001);

        offset += m_settings.configuration.acquisitionInput.numberOfSamplesPerCode;
    }
}
}
