#include "Navigation.h"

#include <QDebug>

GPSOpenCl::Navigation::Navigation(std::vector<double> navInput, QObject *parent) : QThread(parent)
{
    std::copy(navInput.begin(), navInput.end(), std::back_inserter(m_input));
    m_logger = new Logger();
}

GPSOpenCl::Navigation::~Navigation()
{
}

void GPSOpenCl::Navigation::run()
{
    checkPreamble();
}

void GPSOpenCl::Navigation::checkPreamble()
{
    std::vector<int> preamble{1, -1, -1, -1, 1, -1, 1, 1};
    int bitsLength = m_input.size();

    int preambleLength = 160;
    std::vector<double> preambleUpsampled;

    int cnt = 0;
    for (int i = 0; i < preambleLength; i++)
    {
        if ((i != 0) && (i % 20 == 0))
        {
            cnt++;
        }
        preambleUpsampled.push_back(preamble[cnt]);
    }

    std::vector<double> bits;
    for (int i = 0; i < bitsLength; i++)
    {
        if (m_input[i] > 0)
        {
            bits.push_back(1.0);
        }
        else
        {
            bits.push_back(-1.0);
        }
    }
    std::vector<double> convVec = convolve(bits, preambleUpsampled);
    m_logger->log(convVec);
}

std::vector<double> GPSOpenCl::Navigation::convolve(std::vector<double> input, std::vector<double> kernel)
{
    std::reverse(kernel.begin(), kernel.end());

    int inputLength = input.size();
    int kernelLength = kernel.size();
    int outputLength = inputLength + kernelLength - 1;
    std::vector<double> output(outputLength);

    for (int i = 0; i < outputLength; i++)
    {
        output[i] = 0;
        for (int j = 0; j < kernelLength; j++)
        {
            if (i - j >= 0)
            {
                output[i] = output[i] + input[j] * kernel[i - j];
            }
            else
            {
                output[i] = output[i] + input[j] * kernel[i - j + kernelLength];
            }
        }
    }
    return output;
}
