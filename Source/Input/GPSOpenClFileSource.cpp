#include "Input/GPSOpenClFileSource.hpp"
#include <cmath>
#include <fstream>
#include <iostream>

namespace GPSOpenCl
{
FileSource::FileSource() = default;

FileSource::~FileSource() = default;

bool FileSource::initialize(const SourceInput &input)
{
    m_inputConfig = input;
    m_currentBlockIndex = 0;
    const std::string filePath = input.fifoPath;
    if (filePath.empty())
    {
        return false;
    }
    size_t samplesPerBlock = 4096;
    if (input.samplingRateHz > 0.0)
    {
        samplesPerBlock = static_cast<size_t>(std::round(input.samplingRateHz / (GPS_CA_CODE_FREQUENCY_HZ / GPS_CA_CODE_LENGTH)));
    }

    if (filePath.find(".bin") != std::string::npos)
    {
        m_samplesPerBlock = samplesPerBlock;
        m_binFile.open(filePath, std::ios::binary);
        if (!m_binFile.is_open())
        {
            return false;
        }

        m_binFile.seekg(0, std::ios::end);
        const size_t fileSize = m_binFile.tellg();
        m_binFile.seekg(0, std::ios::beg);

        m_totalSamples = fileSize / 2;
        m_byteBuffer.resize(samplesPerBlock * 2);
        m_streaming = true;
        return m_totalSamples > 0;
    }

    return loadAllSamples(filePath, samplesPerBlock);
}

bool FileSource::loadAllSamples(const std::string &filePath, size_t samplesPerBlock)
{
    m_samplesPerBlock = samplesPerBlock;
    m_allSamples.clear();
    m_currentBlockIndex = 0;
    m_streaming = false;

    std::ifstream file(filePath);
    if (!file.is_open())
    {
        return false;
    }

    std::string str;
    int lineCounter = 0;
    size_t fileLine = 0;
    size_t badLineCount = 0;
    float realVal = 0.0f;
    float imagVal = 0.0f;
    while (std::getline(file, str))
    {
        fileLine++;
        if (str.empty())
        {
            continue;
        }
        float parsed = 0.0f;
        try
        {
            parsed = std::stof(str);
        }
        catch (const std::exception &)
        {
            if (badLineCount == 0)
            {
                std::cerr << "FileSource: skipping unparseable sample at " << filePath << ":" << fileLine << " ('" << str << "')" << '\n';
            }
            badLineCount++;
            continue;
        }
        if (lineCounter % 2 == 0)
        {
            realVal = parsed;
        }
        else
        {
            imagVal = parsed;
            m_allSamples.emplace_back(realVal, imagVal);
        }
        lineCounter++;
    }
    if (badLineCount > 0)
    {
        std::cerr << "FileSource: skipped " << badLineCount << " unparseable line(s) in " << filePath << '\n';
    }

    return !m_allSamples.empty();
}

bool FileSource::readBlock(ComplexFloatVector &outputSamples, SourceOutput &telemetry)
{
    if (m_samplesPerBlock == 0)
    {
        m_samplesPerBlock = 4096;
    }

    if (m_streaming)
    {
        const size_t startIdx = m_currentBlockIndex * m_samplesPerBlock;
        if (startIdx + m_samplesPerBlock > m_totalSamples)
        {
            return false;
        }

        const size_t bytesNeeded = m_samplesPerBlock * 2;
        if (m_byteBuffer.size() < bytesNeeded)
        {
            m_byteBuffer.resize(bytesNeeded);
        }
        m_binFile.read(reinterpret_cast<char *>(m_byteBuffer.data()), static_cast<std::streamsize>(bytesNeeded));
        if (static_cast<size_t>(m_binFile.gcount()) < bytesNeeded)
        {
            return false;
        }

        outputSamples.resize(m_samplesPerBlock);
        for (size_t i = 0; i < m_samplesPerBlock; i++)
        {
            auto re = static_cast<float>(m_byteBuffer[2 * i]);
            auto im = static_cast<float>(m_byteBuffer[(2 * i) + 1]);
            outputSamples[i] = std::complex<float>(re, im);
        }
    }
    else
    {
        const size_t startIdx = m_currentBlockIndex * m_samplesPerBlock;
        if (startIdx + m_samplesPerBlock > m_allSamples.size())
        {
            return false;
        }

        const auto blockBegin = static_cast<std::ptrdiff_t>(startIdx);
        const auto blockEnd = static_cast<std::ptrdiff_t>(startIdx + m_samplesPerBlock);
        outputSamples.assign(m_allSamples.begin() + blockBegin, m_allSamples.begin() + blockEnd);
    }

    telemetry.structVersion = STRUCT_VERSION_1;
    telemetry.blockIndex = static_cast<uint32_t>(m_currentBlockIndex);
    telemetry.timestampSec = static_cast<double>(m_currentBlockIndex) * GPS_CA_CODE_PERIOD_SEC;
    telemetry.fifoUnderrunCount = 0;
    telemetry.fifoOverrunCount = 0;

    m_currentBlockIndex++;

    if (m_sink)
    {
        m_sink->publishSourceOutput(telemetry);
    }

    return true;
}
}
