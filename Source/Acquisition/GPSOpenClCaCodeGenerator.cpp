#include "Acquisition/GPSOpenClCaCodeGenerator.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <iostream>

using namespace GPSOpenCl;

CaCodeGenerator::CaCodeGenerator() = default;

CaCodeGenerator::CaCodeGenerator(const Settings::Configuration &conf)
{
    setConfiguration(conf);
}

CaCodeGenerator::~CaCodeGenerator() = default;

const std::array<std::array<char, GPS_CA_CODE_LENGTH>, GPS_CA_SV_COUNT> &CaCodeGenerator::rawCaCodes()
{
    static const auto table = []
    {
        std::array<std::array<char, GPS_CA_CODE_LENGTH>, GPS_CA_SV_COUNT> codes{};

        char g1[GPS_CA_CODE_LENGTH];
        char g2[GPS_CA_CODE_LENGTH];
        char R1[10];
        char R2[10];
        char c1 = 0;
        char c2 = 0;
        int i = 0;
        int j = 0;

        for (int prn = 1; prn <= GPS_CA_SV_COUNT; prn++)
        {
            const static int DELAY_CHIPS[] = {
                5,   6,   7,    8,   17,  18,  139, 140,  141, 251, 252,  254, 255, 256, 257,  258, 469,  470, 471, 472,
                473, 474, 509,  512, 513, 514, 515, 516,  859, 860, 861,  862, 863, 950, 947,  948, 950,  67,  103, 91,
                19,  679, 225,  625, 946, 638, 161, 1001, 554, 280, 710,  709, 775, 864, 558,  220, 397,  55,  898, 759,
                367, 299, 1018, 729, 695, 780, 801, 788,  732, 34,  320,  327, 389, 407, 525,  405, 221,  761, 260, 326,
                955, 653, 699,  422, 188, 438, 959, 539,  879, 677, 586,  153, 792, 814, 446,  264, 1015, 278, 536, 819,
                156, 957, 159,  712, 885, 461, 248, 713,  126, 807, 279,  122, 197, 693, 632,  771, 467,  647, 203, 145,
                175, 52,  21,   237, 235, 886, 657, 634,  762, 355, 1012, 176, 603, 130, 359,  595, 68,   386, 797, 456,
                499, 883, 307,  127, 211, 121, 118, 163,  628, 853, 484,  289, 811, 202, 1021, 463, 568,  904, 670, 230,
                911, 684, 309,  644, 932, 12,  314, 891,  212, 185, 675,  503, 150, 395, 345,  846, 798,  992, 357, 995,
                877, 112, 144,  476, 193, 109, 445, 291,  87,  399, 292,  901, 339, 208, 711,  189, 263,  537, 663, 942,
                173, 900, 30,   500, 935, 556, 373, 85,   652, 310};

            for (i = 0; i < 10; i++)
            {
                R1[i] = -1;
                R2[i] = -1;
            }

            for (i = 0; i < 1023; i++)
            {
                g1[i] = R1[9];
                g2[i] = R2[9];
                c1 = static_cast<char>(R1[2] * R1[9]);
                c2 = static_cast<char>(R2[1] * R2[2] * R2[5] * R2[7] * R2[8] * R2[9]);
                for (j = 9; j > 0; j--)
                {
                    R1[j] = R1[j - 1];
                    R2[j] = R2[j - 1];
                }
                R1[0] = c1;
                R2[0] = c2;
            }

            for (i = 0, j = 1023 - DELAY_CHIPS[prn - 1]; i < 1023; i++, j++)
            {
                codes[prn - 1][i] = static_cast<char>(-g1[i] * g2[j % 1023]);
            }
        }

        return codes;
    }();
    return table;
}

void CaCodeGenerator::calculateCACode()
{
    const auto &codes = rawCaCodes();
    for (int prn = 0; prn < GPS_CA_SV_COUNT; prn++)
    {
        std::memcpy(caCode[prn], codes[prn].data(), GPS_CA_CODE_LENGTH);
    }
}

void CaCodeGenerator::createLookupTable(SpectrumEngine *gpu)
{
    upsampledCaCode.clear();
    upsampledFreqDomainCaCode.clear();

    if (m_sampleLength <= 0)
    {
        return;
    }

    std::vector<int> codeIndexes(m_sampleLength, 0);
    float codeValue = 0.0;
    ComplexFloatVector tmpInput;

    for (int i = 1; i <= m_sampleLength; i++)
    {
        codeIndexes[i - 1] =
            static_cast<int>(std::ceil(static_cast<float>(i) * GPS_CA_CODE_FREQUENCY_HZ / m_samplingFrequencyHz));
    }

    codeIndexes[m_sampleLength - 1] = 1023;

    for (int i = 1; i <= GPS_CA_SV_COUNT; i++)
    {
        tmpInput.clear();

        upsampledCaCode.emplace_back();
        upsampledFreqDomainCaCode.emplace_back();

        for (int j = 0; j < m_sampleLength; j++)
        {
            const int codeIdx = std::clamp(codeIndexes[j] - 1, 0, GPS_CA_CODE_LENGTH - 1);
            codeValue = static_cast<float>(caCode[i - 1][codeIdx]);
            upsampledCaCode[i - 1].push_back(codeValue);

            tmpInput.emplace_back(codeValue, 0.0);
        }
        if (gpu->fft(tmpInput, &upsampledFreqDomainCaCode[i - 1], SpectrumEngine::FFTForward) != 0)
        {
            std::cerr << "CaCodeGenerator::createLookupTable: FFT failed for PRN " << i
                      << " (samplesPerCode=" << m_sampleLength
                      << " is not a power of two); aborting lookup table construction." << '\n';
            upsampledCaCode.clear();
            upsampledFreqDomainCaCode.clear();
            return;
        }

        for (int j = 0; j < m_sampleLength; j++)
        {
            const std::complex<float> oldFreqDom = upsampledFreqDomainCaCode[i - 1].at(j);
            const std::complex<float> conjComplexVal =
                std::complex<float>(std::real(oldFreqDom), std::imag(oldFreqDom) * -1.0f);
            upsampledFreqDomainCaCode[i - 1].at(j) = conjComplexVal;
        }
    }
}

void CaCodeGenerator::setConfiguration(const Settings::Configuration &conf)
{
    m_sampleLength = conf.acquisitionInput.numberOfSamplesPerCode;
    m_samplingFrequencyHz = static_cast<float>(conf.acquisitionInput.samplingFrequencyHz);
    initialize();
}

void CaCodeGenerator::initialize()
{
    calculateCACode();
}
