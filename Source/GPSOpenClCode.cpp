#include "GPSOpenClCode.h"

#include <cstring>

using namespace GPSOpenCl;

Code::Code()
{
    m_sampleLength = 0;
    m_samplingFrequencyHz = 0.0f;
}

/**
 * @brief Construct a new GPSOpenCl::Code::Code object
 *
 */
Code::Code(Settings::Configuration conf)
{
    setConfiguration(conf);
}

/**
 * @brief Destroy the GPSOpenCl::Code::Code object
 *
 */
Code::~Code()
{
}

/**
 * @brief
 *
 */
void Code::calculateCACode()
{
    char G1[GPS_CA_CODE_LENGTH];
    char G2[GPS_CA_CODE_LENGTH];
    char R1[10];
    char R2[10];
    char C1;
    char C2;
    int i = 0;
    int j = 0;

    for (int prn = 1; prn <= GPS_CA_SV_COUNT; prn++)
    {
        const static int delayChips[] = {
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
            G1[i] = R1[9];
            G2[i] = R2[9];
            C1 = R1[2] * R1[9];
            C2 = R2[1] * R2[2] * R2[5] * R2[7] * R2[8] * R2[9];
            for (j = 9; j > 0; j--)
            {
                R1[j] = R1[j - 1];
                R2[j] = R2[j - 1];
            }
            R1[0] = C1;
            R2[0] = C2;
        }

        for (i = 0, j = 1023 - delayChips[prn - 1]; i < 1023; i++, j++)
        {
            m_caCode[prn - 1][i] = -G1[i] * G2[j % 1023];
        }
    }
}

/**
 * @brief
 *
 * @param m_sampleLength
 * @param samplingFrequencyHz
 */
void Code::createLookupTable(Compute *gpu)
{
    int codeIndexes[m_sampleLength];
    float codeValue = 0.0;
    ComplexFloatVector codeFreqDomain;
    ComplexFloatVector tmpInput;

    memset(codeIndexes, 0, m_sampleLength * sizeof(int));

    for (int i = 1; i <= m_sampleLength; i++)
    {
        codeIndexes[i - 1] = static_cast<int>(std::ceil(i * GPS_CA_CODE_FREQUENCY_HZ / m_samplingFrequencyHz));
    }

    codeIndexes[m_sampleLength - 1] = 1023;

    for (int i = 1; i <= GPS_CA_SV_COUNT; i++)
    {
        if (!tmpInput.empty())
        {
            tmpInput.clear();
        }

        m_upsampledCaCode.push_back(FloatVector());
        m_upsampledFreqDomainCaCode.push_back(ComplexFloatVector());

        for (int j = 0; j < m_sampleLength; j++)
        {
            codeValue = static_cast<float>(m_caCode[i - 1][codeIndexes[j] - 1]);
            m_upsampledCaCode[i - 1].push_back(codeValue);

            tmpInput.push_back(std::complex<float>(codeValue, 0.0));
        }
        gpu->fft(tmpInput, &m_upsampledFreqDomainCaCode[i - 1], Compute::FFTForward);

        for (int j = 0; j < m_sampleLength; j++)
        {
            std::complex<float> oldFreqDom = m_upsampledFreqDomainCaCode[i - 1].at(j);
            std::complex<float> conjComplexVal =
                std::complex<float>(std::real(oldFreqDom), std::imag(oldFreqDom) * -1.0f);
            m_upsampledFreqDomainCaCode[i - 1].at(j) = conjComplexVal;
        }
    }
}

void Code::setConfiguration(Settings::Configuration conf)
{
    m_sampleLength = conf.rawDataSettings.numberOfSamplesPerCode;
    m_samplingFrequencyHz = conf.rawDataSettings.samplingFrequency;
    initialize();
}

/**
 * @brief
 *
 */
void Code::initialize()
{
    calculateCACode();
}
