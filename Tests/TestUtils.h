#ifndef INCLUDED_TESTUTILS_H
#define INCLUDED_TESTUTILS_H

#include "gtest/gtest.h"

#include "GPSOpenClCommon.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
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
        if (!file.is_open()) return;

        std::string str;
        int lineCounter = 0;
        float realVal = 0.0;
        float imagVal = 0.0;
        std::complex<float> cpxVal;
        while (std::getline(file, str))
        {
            if (!str.empty())
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
                lineCounter++;
            }
        }
    }

    static void readFromFileBinaryIQ8(const char *fileName, GPSOpenCl::ComplexFloatVector *inputVec, size_t maxSamples = 0)
    {
        std::ifstream file(fileName, std::ios::binary);
        if (!file.is_open()) return;

        file.seekg(0, std::ios::end);
        size_t fileSize = file.tellg();
        file.seekg(0, std::ios::beg);

        size_t numSamples = fileSize / 2;
        if (maxSamples > 0 && numSamples > maxSamples)
        {
            numSamples = maxSamples;
        }

        inputVec->resize(numSamples);
        std::vector<int8_t> buffer(numSamples * 2);
        file.read(reinterpret_cast<char *>(buffer.data()), numSamples * 2);

        for (size_t i = 0; i < numSamples; i++)
        {
            float re = static_cast<float>(buffer[2 * i]);
            float im = static_cast<float>(buffer[2 * i + 1]);
            (*inputVec)[i] = std::complex<float>(re, im);
        }
    }

    static void readFromFileBinaryIQ16(const char *fileName, GPSOpenCl::ComplexFloatVector *inputVec, size_t maxSamples = 0)
    {
        std::ifstream file(fileName, std::ios::binary);
        if (!file.is_open()) return;

        file.seekg(0, std::ios::end);
        size_t fileSize = file.tellg();
        file.seekg(0, std::ios::beg);

        size_t numSamples = fileSize / 4;
        if (maxSamples > 0 && numSamples > maxSamples)
        {
            numSamples = maxSamples;
        }

        inputVec->resize(numSamples);
        std::vector<int16_t> buffer(numSamples * 2);
        file.read(reinterpret_cast<char *>(buffer.data()), numSamples * 2);

        for (size_t i = 0; i < numSamples; i++)
        {
            float re = static_cast<float>(buffer[2 * i]);
            float im = static_cast<float>(buffer[2 * i + 1]);
            (*inputVec)[i] = std::complex<float>(re, im);
        }
    }

    static void readFromFileReal(const char *fileName, GPSOpenCl::FloatVector *inputVec)
    {
        std::ifstream file(fileName);
        if (!file.is_open()) return;

        std::string str;
        float value = 0.0;
        while (std::getline(file, str))
        {
            if (!str.empty())
            {
                value = std::stof(str);
                inputVec->push_back(value);
            }
        }
    }

    static std::chrono::steady_clock::time_point startElapsedTimeMeasurement()
    {
        return std::chrono::steady_clock::now();
    }

    static void measureElapsedTime(std::string functionDescription, std::chrono::steady_clock::time_point timeBegin)
    {
        std::chrono::steady_clock::time_point timeEnd = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::nanoseconds>(timeEnd - timeBegin).count();
        std::cout << "Elapsed time for " << functionDescription << " : " << elapsedTime << "[ns]" << std::endl;
    }

    static void compareComplexResults(const GPSOpenCl::ComplexFloatVector &testOutputVec,
                                      const GPSOpenCl::ComplexFloatVector &expectedOutputVec, double tolerance)
    {
        auto sizeOfOutput = testOutputVec.size();
        auto sizeOfExpectedOutput = expectedOutputVec.size();

        EXPECT_EQ(sizeOfExpectedOutput, sizeOfOutput);

        for (size_t i = 0; i < testOutputVec.size(); i++)
        {
            float realExpectedOutput = std::real(expectedOutputVec.at(i));
            float realOutput = std::real(testOutputVec.at(i));
            float imagExpectedInput = std::imag(expectedOutputVec.at(i));
            float imagOutput = std::imag(testOutputVec.at(i));

            compareRealResults(realOutput, realExpectedOutput, tolerance);
            compareRealResults(imagOutput, imagExpectedInput, tolerance);
        }
    }

    static void compareRealResults(const GPSOpenCl::FloatVector &testOutputVec, const GPSOpenCl::FloatVector &expectedOutputVec,
                                   double tolerance)
    {
        auto sizeOfOutput = testOutputVec.size();
        auto sizeOfExpectedOutput = expectedOutputVec.size();

        EXPECT_EQ(sizeOfExpectedOutput, sizeOfOutput);

        for (size_t i = 0; i < testOutputVec.size(); i++)
        {
            float expectedOutput = expectedOutputVec.at(i);
            float output = testOutputVec.at(i);

            compareRealResults(output, expectedOutput, tolerance);
        }
    }

    static void compareRealResults(float testOutput, float expectedOutput, double tolerance)
    {
        float denom = (std::fabs(expectedOutput) < 0.1f) ? 1.0f : std::fabs(expectedOutput);
        float error = std::fabs(testOutput - expectedOutput) / denom;

        EXPECT_LE(error, tolerance);
    }
};
} // namespace GPSOpenClTest

#endif //! INCLUDED_TESTUTILS_H
