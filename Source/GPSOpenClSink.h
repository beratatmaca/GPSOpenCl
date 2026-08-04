#ifndef INCLUDED_GPSOPENCL_SINK_H
#define INCLUDED_GPSOPENCL_SINK_H

/** @file GPSOpenClSink.h
 *  @brief Abstract telemetry output interface and implementations.
 */

#include "GPSOpenClStructs.h"
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace GPSOpenCl
{
/** @brief Owned copy of one publish() call's identifier and struct bytes, queued for a background
 *   writer thread. Sink implementations that do real I/O (file writes, network sends) should copy
 *   into this on their real-time-facing publish() call and hand the actual I/O to a dedicated
 *   thread, so a slow write never stalls the caller. Uses fixed inline storage so queueing a
 *   message performs no heap allocation on the publishing thread. */
struct SinkMessage
{
    static constexpr size_t MAX_IDENTIFIER_BYTES = 32;    ///< Inline identifier capacity.
    static constexpr size_t MAX_DATA_BYTES = 512;         ///< Inline payload capacity.

    uint16_t identifierLength{0};                         ///< Used bytes of identifier.
    uint16_t dataLength{0};                               ///< Used bytes of data.
    char identifier[MAX_IDENTIFIER_BYTES];                ///< Topic/module name (not NUL-terminated).
    char data[MAX_DATA_BYTES];                            ///< Copied struct bytes.

    /** @brief Fill from a publish() call, truncating nothing: returns false if either field does
     *   not fit, in which case the message must not be queued.
     *  @param id   Topic/module name.
     *  @param src  Pointer to struct data.
     *  @param size Size of data (bytes).
     *  @return True if the message fits the inline storage. */
    bool fill(const std::string &id, const void *src, size_t size)
    {
        if (id.size() > MAX_IDENTIFIER_BYTES || size > MAX_DATA_BYTES)
        {
            return false;
        }
        identifierLength = static_cast<uint16_t>(id.size());
        dataLength = static_cast<uint16_t>(size);
        std::memcpy(identifier, id.data(), id.size());
        std::memcpy(data, src, size);
        return true;
    }
};

static_assert(sizeof(SourceOutput) <= SinkMessage::MAX_DATA_BYTES, "SourceOutput exceeds SinkMessage capacity");
static_assert(sizeof(AcquisitionOutput) <= SinkMessage::MAX_DATA_BYTES,
              "AcquisitionOutput exceeds SinkMessage capacity");
static_assert(sizeof(TrackingOutput) <= SinkMessage::MAX_DATA_BYTES, "TrackingOutput exceeds SinkMessage capacity");
static_assert(sizeof(NavDecoderOutput) <= SinkMessage::MAX_DATA_BYTES, "NavDecoderOutput exceeds SinkMessage capacity");
static_assert(sizeof(PvtSolverOutput) <= SinkMessage::MAX_DATA_BYTES, "PvtSolverOutput exceeds SinkMessage capacity");
static_assert(sizeof(AtmosphericOutput) <= SinkMessage::MAX_DATA_BYTES,
              "AtmosphericOutput exceeds SinkMessage capacity");
static_assert(sizeof(NmeaGeneratorOutput) <= SinkMessage::MAX_DATA_BYTES,
              "NmeaGeneratorOutput exceeds SinkMessage capacity");
static_assert(sizeof(ProfilerOutput) <= SinkMessage::MAX_DATA_BYTES, "ProfilerOutput exceeds SinkMessage capacity");

/** @brief Abstract telemetry output interface. Publish calls are serialized by this base class,
 *   so concrete implementations (including test doubles) don't each need their own locking even
 *   when multiple satellite channels publish concurrently from the tracking worker pool. */
class Sink
{
  public:
    virtual ~Sink() = default;
    Sink() = default;
    Sink(const Sink &) = delete;
    Sink &operator=(const Sink &) = delete;
    Sink(Sink &&) = delete;
    Sink &operator=(Sink &&) = delete;

    /** @brief Publish binary data with an identifier.
     *  @param identifier Topic/module name.
     *  @param data       Pointer to struct data.
     *  @param size       Size of data (bytes). */
    virtual void publish(const std::string &identifier, const void *data, size_t size) = 0;

    /** @brief Publish SourceOutput telemetry.
     *  @param out Source output struct. */
    void publishSourceOutput(const SourceOutput &out)
    {
        const std::lock_guard<std::mutex> lock(m_publishMutex);
        publish("SourceOutput", &out, sizeof(out));
    }

    /** @brief Publish AcquisitionOutput telemetry.
     *  @param out Acquisition output struct. */
    void publishAcquisitionOutput(const AcquisitionOutput &out)
    {
        const std::lock_guard<std::mutex> lock(m_publishMutex);
        publish("AcquisitionOutput", &out, sizeof(out));
    }

    /** @brief Publish TrackingOutput telemetry.
     *  @param out Tracking output struct. */
    void publishTrackingOutput(const TrackingOutput &out)
    {
        const std::lock_guard<std::mutex> lock(m_publishMutex);
        publish("TrackingOutput", &out, sizeof(out));
    }

    /** @brief Publish NavDecoderOutput telemetry.
     *  @param out Nav decoder output struct. */
    void publishNavDecoderOutput(const NavDecoderOutput &out)
    {
        const std::lock_guard<std::mutex> lock(m_publishMutex);
        publish("NavDecoderOutput", &out, sizeof(out));
    }

    /** @brief Publish PvtSolverOutput telemetry.
     *  @param out PVT solver output struct. */
    void publishPvtSolverOutput(const PvtSolverOutput &out)
    {
        const std::lock_guard<std::mutex> lock(m_publishMutex);
        publish("PvtSolverOutput", &out, sizeof(out));
    }

    /** @brief Publish AtmosphericOutput telemetry.
     *  @param out Atmospheric output struct. */
    void publishAtmosphericOutput(const AtmosphericOutput &out)
    {
        const std::lock_guard<std::mutex> lock(m_publishMutex);
        publish("AtmosphericOutput", &out, sizeof(out));
    }

    /** @brief Publish NmeaGeneratorOutput telemetry.
     *  @param out NMEA output struct. */
    void publishNmeaGeneratorOutput(const NmeaGeneratorOutput &out)
    {
        const std::lock_guard<std::mutex> lock(m_publishMutex);
        publish("NmeaGeneratorOutput", &out, sizeof(out));
    }

    /** @brief Publish ProfilerOutput telemetry.
     *  @param out Profiler output struct. */
    void publishProfilerOutput(const ProfilerOutput &out)
    {
        const std::lock_guard<std::mutex> lock(m_publishMutex);
        publish("ProfilerOutput", &out, sizeof(out));
    }

  private:
    std::mutex m_publishMutex;    ///< Serializes publish() calls across concurrent callers.
};

/** @brief No-op sink that discards all telemetry. */
class NullSink : public Sink
{
  public:
    /** @brief Discard published data. */
    void publish(const std::string &, const void *, size_t) override {}
};

/** @brief Fan-out sink that forwards to multiple downstream sinks. */
class CompositeSink : public Sink
{
  public:
    /** @brief Add a downstream sink.
     *  @param sink Sink to add. */
    void addSink(const std::shared_ptr<Sink> &sink)
    {
        if (sink)
        {
            m_sinks.push_back(sink);
        }
    }

    /** @brief Forward data to all downstream sinks. */
    void publish(const std::string &identifier, const void *data, size_t size) override
    {
        for (auto &s : m_sinks)
        {
            s->publish(identifier, data, size);
        }
    }

  private:
    std::vector<std::shared_ptr<Sink>> m_sinks;    ///< Downstream sinks.
};

}

#endif
