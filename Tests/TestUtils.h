#ifndef INCLUDED_TESTUTILS_H
#define INCLUDED_TESTUTILS_H

#include "gtest/gtest.h"

#include "../Source/GPSOpenClCommon.h"

#include <chrono>
#include <fstream>
#include <string>

namespace GPSOpenClTest
{
class TestUtils
{
  public:
    TestUtils(){};

    ~TestUtils(){};

    static void readFromFileComplex(const char *fileName, GPSOpenCl::ComplexFloatVector *inputVec)
    {
        std::ifstream file(fileName);
        std::string str;
        int lineCounter = 0;
        float realVal = 0.0;
        float imagVal = 0.0;
        std::complex<float> cpxVal;
        while (std::getline(file, str))
        {
            if (str.size() > 0)
            {
                if (lineCounter % 2 == 0)
                {
                    realVal = std::stof(str);
                }
                else
                {
                    imagVal = std::stof(str);
                    cpxVal = std::complex<float>(realVal, imagVal);
                    inputVec->push_back(cpxVal);
                }
            }
            else
            {
            }
            lineCounter++;
        }
    }

    static void readFromFileReal(const char *fileName, GPSOpenCl::FloatVector *inputVec)
    {
        std::ifstream file(fileName);
        std::string str;
        float value = 0.0;
        int lineCounter = 0;
        while (std::getline(file, str))
        {
            if (str.size() > 0)
            {
                value = std::stof(str);
                inputVec->push_back(value);
            }
            else
            {
            }
            lineCounter++;
        }
    }

    static std::chrono::steady_clock::time_point startElapsedTimeMeasurement()
    {
        std::chrono::steady_clock::time_point timeBegin = std::chrono::steady_clock::now();
        return timeBegin;
    }

    static void measureElapsedTime(std::string functionDescription, std::chrono::steady_clock::time_point timeBegin)
    {

        std::chrono::steady_clock::time_point timeEnd = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::nanoseconds>(timeEnd - timeBegin).count();
        std::cout << "Elapsed time for " << functionDescription << " : " << elapsedTime << "[ns]" << std::endl;
    }

    static void compareComplexResults(GPSOpenCl::ComplexFloatVector testOutputVec,
                                      GPSOpenCl::ComplexFloatVector expectedOutputVec, double tolerance)
    {
        auto sizeOfOutput = testOutputVec.size();
        auto sizeOfExpectedOutput = expectedOutputVec.size();

        EXPECT_EQ(sizeOfExpectedOutput, sizeOfOutput);

        for (int i = 0; i < static_cast<int>(testOutputVec.size()); i++)
        {
            float realExpectedOutput = std::real(expectedOutputVec.at(i));
            float realOutput = std::real(testOutputVec.at(i));
            float imagExpectedInput = std::imag(expectedOutputVec.at(i));
            float imagOutput = std::imag(testOutputVec.at(i));

            compareRealResults(realExpectedOutput, realOutput, tolerance);
            compareRealResults(imagExpectedInput, imagOutput, tolerance);
        }
    }

    static void compareRealResults(GPSOpenCl::FloatVector testOutputVec, GPSOpenCl::FloatVector expectedOutputVec,
                                   double tolerance)
    {
        auto sizeOfOutput = testOutputVec.size();
        auto sizeOfExpectedOutput = expectedOutputVec.size();

        EXPECT_EQ(sizeOfExpectedOutput, sizeOfOutput);

        for (int i = 0; i < static_cast<int>(testOutputVec.size()); i++)
        {
            float expectedOutput = expectedOutputVec.at(i);
            float output = testOutputVec.at(i);

            compareRealResults(expectedOutput, output, tolerance);
        }
    }

    static void compareRealResults(float testOutput, float expectedOutput, double tolerance)
    {
        float percentage = 0.0f;

        if (expectedOutput != 0.0f)
        {
            percentage = std::fabs((testOutput / expectedOutput) - 1.0f);
        }
        else
        {
            percentage = std::fabs(testOutput);
        }

        EXPECT_LE(percentage, tolerance);
    }
};
} // namespace GPSOpenClTest

#endif //! INCLUDED_TESTUTILS_H
