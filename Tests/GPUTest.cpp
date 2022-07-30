#include "../Source/GPSOpenClGPUCompute.h"

#include "TestUtils.h"

#define FFT_TOLERANCE 0.3f
#define MULTIPLICATION_TOLERANCE 0.01f

namespace GPSOpenClTest
{
class GPUTest : public testing::Test
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

TEST_F(GPUTest, FFTTest1)
{
    GPSOpenCl::ComplexFloatVector testInputVec;
    GPSOpenCl::ComplexFloatVector testOutputVec;
    GPSOpenCl::ComplexFloatVector expectedOutputVec;

    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/FFTTestCase1Input.txt", &testInputVec);
    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/FFTTestCase1Output.txt", &expectedOutputVec);

    auto timeBegin = TestUtils::startElapsedTimeMeasurement();

    m_gpuCompute.fft(testInputVec, &testOutputVec, GPSOpenCl::Compute::FFTForward);

    TestUtils::measureElapsedTime("FFT function for 2048 points", timeBegin);

    TestUtils::compareComplexResults(testOutputVec, expectedOutputVec, FFT_TOLERANCE);
}

TEST_F(GPUTest, FFTTest2)
{
    GPSOpenCl::ComplexFloatVector testInputVec;
    GPSOpenCl::ComplexFloatVector testOutputVec;
    GPSOpenCl::ComplexFloatVector expectedOutputVec;

    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/FFTTestCase2Input.txt", &testInputVec);
    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/FFTTestCase2Output.txt", &expectedOutputVec);

    auto timeBegin = TestUtils::startElapsedTimeMeasurement();

    m_gpuCompute.fft(testInputVec, &testOutputVec, GPSOpenCl::Compute::FFTForward);

    TestUtils::measureElapsedTime("FFT function for 4096 points", timeBegin);

    TestUtils::compareComplexResults(testOutputVec, expectedOutputVec, FFT_TOLERANCE);
}

TEST_F(GPUTest, FFTTest3)
{
    GPSOpenCl::ComplexFloatVector testInputVec;
    GPSOpenCl::ComplexFloatVector testOutputVec;
    GPSOpenCl::ComplexFloatVector expectedOutputVec;

    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/FFTTestCase3Input.txt", &testInputVec);
    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/FFTTestCase3Output.txt", &expectedOutputVec);

    auto timeBegin = TestUtils::startElapsedTimeMeasurement();

    m_gpuCompute.fft(testInputVec, &testOutputVec, GPSOpenCl::Compute::FFTForward);

    TestUtils::measureElapsedTime("FFT function for 8192 points", timeBegin);

    TestUtils::compareComplexResults(testOutputVec, expectedOutputVec, FFT_TOLERANCE);
}

TEST_F(GPUTest, FFTTest4)
{
    GPSOpenCl::ComplexFloatVector testInputVec;
    GPSOpenCl::ComplexFloatVector testOutputVec;
    GPSOpenCl::ComplexFloatVector expectedOutputVec;

    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/FFTTestCase4Input.txt", &testInputVec);
    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/FFTTestCase4Output.txt", &expectedOutputVec);

    auto timeBegin = TestUtils::startElapsedTimeMeasurement();

    m_gpuCompute.fft(testInputVec, &testOutputVec, GPSOpenCl::Compute::FFTForward);

    TestUtils::measureElapsedTime("FFT function for 16384 points", timeBegin);

    TestUtils::compareComplexResults(testOutputVec, expectedOutputVec, FFT_TOLERANCE);
}

TEST_F(GPUTest, IFFTTest1)
{
    GPSOpenCl::ComplexFloatVector testInputVec;
    GPSOpenCl::ComplexFloatVector testOutputVec;
    GPSOpenCl::ComplexFloatVector expectedOutputVec;

    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/IFFTTestCase1Input.txt", &testInputVec);
    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/IFFTTestCase1Output.txt", &expectedOutputVec);

    auto timeBegin = TestUtils::startElapsedTimeMeasurement();

    m_gpuCompute.fft(testInputVec, &testOutputVec, GPSOpenCl::Compute::FFTInverse);

    TestUtils::measureElapsedTime("IFFT function for 2048 points", timeBegin);

    TestUtils::compareComplexResults(testOutputVec, expectedOutputVec, FFT_TOLERANCE);
}

TEST_F(GPUTest, IFFTTest2)
{
    GPSOpenCl::ComplexFloatVector testInputVec;
    GPSOpenCl::ComplexFloatVector testOutputVec;
    GPSOpenCl::ComplexFloatVector expectedOutputVec;

    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/IFFTTestCase2Input.txt", &testInputVec);
    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/IFFTTestCase2Output.txt", &expectedOutputVec);

    auto timeBegin = TestUtils::startElapsedTimeMeasurement();

    m_gpuCompute.fft(testInputVec, &testOutputVec, GPSOpenCl::Compute::FFTInverse);

    TestUtils::measureElapsedTime("IFFT function for 4096 points", timeBegin);

    TestUtils::compareComplexResults(testOutputVec, expectedOutputVec, FFT_TOLERANCE);
}

TEST_F(GPUTest, IFFTTest3)
{
    GPSOpenCl::ComplexFloatVector testInputVec;
    GPSOpenCl::ComplexFloatVector testOutputVec;
    GPSOpenCl::ComplexFloatVector expectedOutputVec;

    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/IFFTTestCase3Input.txt", &testInputVec);
    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/IFFTTestCase3Output.txt", &expectedOutputVec);

    auto timeBegin = TestUtils::startElapsedTimeMeasurement();

    m_gpuCompute.fft(testInputVec, &testOutputVec, GPSOpenCl::Compute::FFTInverse);

    TestUtils::measureElapsedTime("IFFT function for 8192 points", timeBegin);

    TestUtils::compareComplexResults(testOutputVec, expectedOutputVec, FFT_TOLERANCE);
}

TEST_F(GPUTest, IFFTTest4)
{
    GPSOpenCl::ComplexFloatVector testInputVec;
    GPSOpenCl::ComplexFloatVector testOutputVec;
    GPSOpenCl::ComplexFloatVector expectedOutputVec;

    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/IFFTTestCase4Input.txt", &testInputVec);
    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/IFFTTestCase4Output.txt", &expectedOutputVec);

    auto timeBegin = TestUtils::startElapsedTimeMeasurement();

    m_gpuCompute.fft(testInputVec, &testOutputVec, GPSOpenCl::Compute::FFTInverse);

    TestUtils::measureElapsedTime("IFFT function for 16384 points", timeBegin);

    TestUtils::compareComplexResults(testOutputVec, expectedOutputVec, FFT_TOLERANCE);
}
TEST_F(GPUTest, ComplexMultiplicationTest1)
{
    GPSOpenCl::ComplexFloatVector testInputVec1;
    GPSOpenCl::ComplexFloatVector testInputVec2;
    GPSOpenCl::ComplexFloatVector testOutputVec;
    GPSOpenCl::ComplexFloatVector expectedOutputVec;

    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/ComplexMultiplierTestCase1Input1.txt", &testInputVec1);
    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/ComplexMultiplierTestCase1Input2.txt", &testInputVec2);
    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/ComplexMultiplierTestCase1Output.txt",
                                   &expectedOutputVec);

    auto timeBegin = TestUtils::startElapsedTimeMeasurement();

    m_gpuCompute.complexMultiplier(testInputVec1, testInputVec2, &testOutputVec);

    TestUtils::measureElapsedTime("Complex Multiplication for 4096 points", timeBegin);

    TestUtils::compareComplexResults(testOutputVec, expectedOutputVec, MULTIPLICATION_TOLERANCE);
}
TEST_F(GPUTest, ComplexMultiplicationTest2)
{
    GPSOpenCl::ComplexFloatVector testInputVec1;
    GPSOpenCl::ComplexFloatVector testInputVec2;
    GPSOpenCl::ComplexFloatVector testOutputVec;
    GPSOpenCl::ComplexFloatVector expectedOutputVec;

    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/ComplexMultiplierTestCase2Input1.txt", &testInputVec1);
    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/ComplexMultiplierTestCase2Input2.txt", &testInputVec2);
    TestUtils::readFromFileComplex("../../Tests/Scripts/GPUTest/ComplexMultiplierTestCase2Output.txt",
                                   &expectedOutputVec);

    auto timeBegin = TestUtils::startElapsedTimeMeasurement();

    m_gpuCompute.complexMultiplier(testInputVec1, testInputVec2, &testOutputVec);

    TestUtils::measureElapsedTime("Complex Multiplication for 8192 points", timeBegin);

    TestUtils::compareComplexResults(testOutputVec, expectedOutputVec, MULTIPLICATION_TOLERANCE);
}
} // namespace GPSOpenClTest
