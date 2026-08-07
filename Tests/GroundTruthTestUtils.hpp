#ifndef INCLUDED_GPSOPENCLTEST_GROUNDTRUTHTESTUTILS_HPP
#define INCLUDED_GPSOPENCLTEST_GROUNDTRUTHTESTUTILS_HPP

/** @file GroundTruthTestUtils.h
 *  @brief Shared ground-truth test scaffolding: a Sink that captures every telemetry struct type,
 *   and a reader for gps-sdr-sim's -G ground-truth export file.
 */

#include "Common/GPSOpenClStructs.hpp"
#include "GroundTruthRecord.hpp"
#include "Sink/GPSOpenClSink.hpp"

#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace GPSOpenClTest
{
/** @brief Sink that captures every published telemetry struct into a per-type vector, for
 *   comparing the receiver's own reported output against simulator ground truth after a run. */
class CapturingSink : public GPSOpenCl::Sink
{
  public:
    std::vector<GPSOpenCl::AcquisitionOutput> acquisitionOutputs;    ///< Every published AcquisitionOutput, in order.
    std::vector<GPSOpenCl::TrackingOutput> trackingOutputs;          ///< Every published TrackingOutput, in order.
    std::vector<GPSOpenCl::NavDecoderOutput> navDecoderOutputs;      ///< Every published NavDecoderOutput, in order.
    std::vector<GPSOpenCl::PvtSolverOutput> pvtSolverOutputs;        ///< Every published PvtSolverOutput, in order.

    void publish(const std::string &identifier, const void *data, size_t size) override
    {
        if (identifier == "AcquisitionOutput" && size == sizeof(GPSOpenCl::AcquisitionOutput))
        {
            GPSOpenCl::AcquisitionOutput out;
            std::memcpy(&out, data, sizeof(out));
            acquisitionOutputs.push_back(out);
        }
        else if (identifier == "TrackingOutput" && size == sizeof(GPSOpenCl::TrackingOutput))
        {
            GPSOpenCl::TrackingOutput out;
            std::memcpy(&out, data, sizeof(out));
            trackingOutputs.push_back(out);
        }
        else if (identifier == "NavDecoderOutput" && size == sizeof(GPSOpenCl::NavDecoderOutput))
        {
            GPSOpenCl::NavDecoderOutput out;
            std::memcpy(&out, data, sizeof(out));
            navDecoderOutputs.push_back(out);
        }
        else if (identifier == "PvtSolverOutput" && size == sizeof(GPSOpenCl::PvtSolverOutput))
        {
            GPSOpenCl::PvtSolverOutput out;
            std::memcpy(&out, data, sizeof(out));
            pvtSolverOutputs.push_back(out);
        }
    }
};

/** @brief Read every GroundTruthRecord from a gps-sdr-sim -G export file.
 *  @param path    File path.
 *  @param records Output records, appended in file order.
 *  @return True if the file was opened successfully. */
inline bool readGroundTruth(const std::string &path, std::vector<GroundTruthRecord> *records)
{
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open())
    {
        return false;
    }

    GroundTruthRecord record;
    while (file.read(reinterpret_cast<char *>(&record), sizeof(record)))
    {
        records->push_back(record);
    }
    return true;
}
}

#endif    //! INCLUDED_GPSOPENCLTEST_GROUNDTRUTHTESTUTILS_HPP
