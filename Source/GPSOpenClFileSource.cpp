#include "GPSOpenClFileSource.h"
#include <cmath>
#include <fstream>
#include <iostream>

namespace GPSOpenCl
{
FileSource::FileSource() : m_currentBlockIndex(0), m_samplesPerBlock(4096)
{
}

FileSource::~FileSource()
{
}

bool FileSource::initialize(const SourceInput &input)
{
    m_inputConfig = input;
    m_currentBlockIndex = 0;
    std::string filePath = input.fifoPath;
    if (filePath.empty())
    {
        return false;
    }
    return loadAllSamples(filePath, 4096);
}

bool FileSource::loadAllSamples(const std::string &filePath, size_t samplesPerBlock)
{
    m_samplesPerBlock = samplesPerBlock;
    m_allSamples.clear();
    m_currentBlockIndex = 0;

    if (filePath.find(".bin") != std::string::npos)
    {
        std::ifstream file(filePath, std::ios::binary);
        if (!file.is_open()) return false;

        file.seekg(0, std::ios::end);
        size_t fileSize = file.tellg();
        file.seekg(0, std::ios::beg);

        size_t numSamples = fileSize / 2;
        m_allSamples.resize(numSamples);
        std::vector<int8_t> buffer(numSamples * 2);
        file.read(reinterpret_cast<char *>(buffer.data()), numSamples * 2);

        for (size_t i = 0; i < numSamples; i++)
        {
            float re = static_cast<float>(buffer[2 * i]);
            float im = static_cast<float>(buffer[2 * i + 1]);
            m_allSamples[i] = std::complex<float>(re, im);
        }
    }
    else
    {
        std::ifstream file(filePath);
        if (!file.is_open()) return false;

        std::string str;
        int lineCounter = 0;
        float realVal = 0.0f;
        float imagVal = 0.0f;
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
                    m_allSamples.push_back(std::complex<float>(realVal, imagVal));
                }
                lineCounter++;
            }
        }
    }

    return !m_allSamples.empty();
}

bool FileSource::readBlock(ComplexFloatVector &outputSamples, SourceOutput &telemetry)
{
    if (m_samplesPerBlock == 0) m_samplesPerBlock = 4096;
    size_t startIdx = m_currentBlockIndex * m_samplesPerBlock;
    if (startIdx + m_samplesPerBlock > m_allSamples.size())
    {
        return false;
    }

    outputSamples.assign(m_allSamples.begin() + startIdx, m_allSamples.begin() + startIdx + m_samplesPerBlock);

    telemetry.structVersion = STRUCT_VERSION_1;
    telemetry.blockIndex = static_cast<uint32_t>(m_currentBlockIndex);
    telemetry.timestamp = static_cast<double>(m_currentBlockIndex) * 0.001;
    telemetry.fifoUnderrunCount = 0;
    telemetry.fifoOverrunCount = 0;

    m_currentBlockIndex++;

    if (m_sink)
    {
        m_sink->publishSourceOutput(telemetry);
    }

    return true;
}
} // namespace GPSOpenCl
