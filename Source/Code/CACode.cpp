#include "CACode.h"

#include <cstdlib> // for malloc()
#include <cmath>   // for ceil()
#include "../Logger/Logger.h"

GPSOpenCl::CACode::CACode()
{
    m_samplingTime = 1.0 / 4096000.0;
    m_codePeriodTime = 1.0 / 1023000.0;
    m_codeResampledLength = 4096;

    createCACodeTable();
}

GPSOpenCl::CACode::~CACode()
{
}

double *GPSOpenCl::CACode::calculateCACode(int prn)
{
    const static int delayChips[] = {
        5, 6, 7, 8, 17, 18, 139, 140, 141, 251,
        252, 254, 255, 256, 257, 258, 469, 470, 471, 472,
        473, 474, 509, 512, 513, 514, 515, 516, 859, 860,
        861, 862, 863, 950, 947, 948, 950, 67, 103, 91,
        19, 679, 225, 625, 946, 638, 161, 1001, 554, 280,
        710, 709, 775, 864, 558, 220, 397, 55, 898, 759,
        367, 299, 1018, 729, 695, 780, 801, 788, 732, 34,
        320, 327, 389, 407, 525, 405, 221, 761, 260, 326,
        955, 653, 699, 422, 188, 438, 959, 539, 879, 677,
        586, 153, 792, 814, 446, 264, 1015, 278, 536, 819,
        156, 957, 159, 712, 885, 461, 248, 713, 126, 807,
        279, 122, 197, 693, 632, 771, 467, 647, 203, 145,
        175, 52, 21, 237, 235, 886, 657, 634, 762, 355,
        1012, 176, 603, 130, 359, 595, 68, 386, 797, 456,
        499, 883, 307, 127, 211, 121, 118, 163, 628, 853,
        484, 289, 811, 202, 1021, 463, 568, 904, 670, 230,
        911, 684, 309, 644, 932, 12, 314, 891, 212, 185,
        675, 503, 150, 395, 345, 846, 798, 992, 357, 995,
        877, 112, 144, 476, 193, 109, 445, 291, 87, 399,
        292, 901, 339, 208, 711, 189, 263, 537, 663, 942,
        173, 900, 30, 500, 935, 556, 373, 85, 652, 310};

    double *code = (double *)malloc(sizeof(double) * 1024);
    char G1[1023];
    char G2[1023];
    char R1[10];
    char R2[10];
    char C1;
    char C2;
    int i, j;

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
        code[i] = -G1[i] * G2[j % 1023];
    }

    return code;
}

void GPSOpenCl::CACode::createCACodeTable()
{
    int codeIndexes[m_codeResampledLength] = {0};
    for (int i = 1; i <= m_codeResampledLength; i++)
    {
        codeIndexes[i - 1] = static_cast<int>(ceil(i * m_samplingTime / m_codePeriodTime));
    }
    codeIndexes[m_codeResampledLength - 1] = 1023;

    for (int i = 1; i <= 32; i++)
    {
        double *code = calculateCACode(i);
        m_code.push_back(std::vector<std::complex<double>>());
        for (int j = 0; j < m_codeResampledLength; j++)
        {
            m_code[i - 1].push_back(std::complex<double>(code[codeIndexes[j] - 1], 0));
        }
        delete code;
    }
}
