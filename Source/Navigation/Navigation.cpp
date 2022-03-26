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
    std::vector<double> preambleIndexes = findPreamble(bits, preambleUpsampled);
    //qDebug() << convVec;
    m_logger->log(preambleIndexes);
}

std::vector<double> GPSOpenCl::Navigation::findPreamble(std::vector<double> input, std::vector<double> kernel)
{
    std::reverse(kernel.begin(), kernel.end());
    int inputLength = input.size();
    int kernelLength = kernel.size();
    for (int i = kernel.size(); i < inputLength; i++)
    {
        kernel.push_back(0);
    }
    int outputLength = inputLength + kernelLength - 1;
    std::vector<double> output;
    
    for (int i = 0; i < outputLength; i++)
    {
        double sum = 0;
        for (int j = 0; j < kernelLength; j++)
        {
            if ((i - j) >= 0)
            {
                sum += input[i - j] * kernel[j];
            }
        }
        if (std::fabs(sum) > 153)
        {
            output.push_back(i - 160);
        }
    }
    return output;
}
