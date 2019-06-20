#include "time_synchronizer.h"

#include <rhoban_utils/timing/time_stamp.h>
#include <debug.h>
#include <data.h>
#include <VisionClient.h>

namespace rhoban_ssl
{
namespace vision
{
CameraTimeSynchronizer::CameraTimeSynchronizer()
{
  for (uint i = 0; i < ai::Config::NB_CAMERAS; ++i)
    last_t_capture_[i] = -1.0;
}

bool CameraTimeSynchronizer::runTask()
{
  using std::chrono::high_resolution_clock;
  high_resolution_clock::time_point now = high_resolution_clock::now();

  int count = 0;
  for (auto& p : VisionDataGlobal::singleton_.last_packets_)
  {
    if (p->has_detection())
    {
      count += 1;
      unsigned int cid = p->detection().camera_id();
      last_t_capture_[cid] = p->detection().t_capture();

      if (cid == 0)
      {
        syncDeltaAiCamera(now.time_since_epoch().count(), last_t_capture_[0]);
      }
    }
  }

  if (count == ai::Config::NB_CAMERAS)
    syncDeltaBetweenCameras();

  return true;
}

void CameraTimeSynchronizer::syncDeltaAiCamera(double current_cpu_time, double camera_time)
{
  double new_delta = current_cpu_time - camera_time;
  if (std::abs(new_delta) < std::abs(Data::get()->time_delta_cameras_to_ai))
  {
    Data::get()->time_delta_cameras_to_ai = new_delta;
  }
}

void CameraTimeSynchronizer::syncDeltaBetweenCameras()
{
  for (uint c = 1; c < ai::Config::NB_CAMERAS; ++c)
  {
    double new_delta = last_t_capture_[0] - last_t_capture_[c];
    if (std::abs(new_delta) < std::abs(Data::get()->time_delta_between_cameras_and_cam_0[c]))
      Data::get()->time_delta_between_cameras_and_cam_0[c] = new_delta;
  }
}

///////////////////////////////////////////////////////////////////////////////

TimeSynchroniser::TimeSynchroniser() : diff_buffer_(BUFFER_SIZE), shift_buffer_(BUFFER_SIZE), computed_time_shift(0)
{
}

void TimeSynchroniser::update(const vision::CameraDetectionFrame& detection_frame)
{
  double now = rhoban_utils::TimeStamp::now().getTimeSec();
  double local_t_sent = detection_frame.t_sent_ - computed_time_shift;
  double diff = now - local_t_sent;

  diff_buffer_.insert(diff);

  double average_diff = average(diff_buffer_);

  bool not_synchronised = average_diff > 3e-1;
  bool synchronisation_in_progress = shift_buffer_.size() > 0;
  if (not_synchronised || synchronisation_in_progress)
  {
    double new_time_shift = local_t_sent - now;
    shift_buffer_.insert(new_time_shift);

    computed_time_shift = average(shift_buffer_);

    bool synchronised = average_diff < 1e-4;  // precision 0.1ms
    if (synchronised)
    {
      DEBUG("Synced with vision clock.");
      shift_buffer_.resize(0);
    }
  }
}

void TimeSynchroniser::syncTimeShift(double* time_shift_with_vision)
{
  *time_shift_with_vision = computed_time_shift;
}

double TimeSynchroniser::average(const CircularVector<double>& buffer)
{
  assert(buffer.size() > 0);

  uint size = buffer.size();
  double avg = 0;
  for (uint i = 0; i < size; ++i)
  {
    avg += buffer[i];
  }
  return avg / size;
}

///////////////////////////////////////////////////////////////////////////////

StatCameraTimeIntervalStability::StatCameraTimeIntervalStability(uint freq)
  : counter_(0)
  , freq_(freq)
  , sum_(0.0)
  , min_(std::numeric_limits<double>::max())
  , max_(std::numeric_limits<double>::min())
  , init(true)
  , last_t_capture_(0.0)
{
}

bool StatCameraTimeIntervalStability::runTask()
{
  for (auto& p : VisionDataGlobal::singleton_.last_packets_)
  {
    if (p->has_detection())
    {
      const SSL_DetectionFrame& detection = p->detection();
      if (detection.camera_id() == 0)
      {
        if (init)
        {
          last_t_capture_ = detection.t_capture();
          init = false;
        }
        else
        {
          if (detection.t_capture() <= last_t_capture_)
          {
            DEBUG("INVALID TIME");
            assert(false);
          }
          double new_t_capture = detection.t_capture();
          double dt = new_t_capture - last_t_capture_;
          last_t_capture_ = new_t_capture;

          sum_ += dt;
          min_ = std::min(abs(dt), abs(min_));
          max_ = std::max(abs(dt), abs(max_));
          counter_ += 1;

          if (counter_ % freq_ == 0)
            std::cout << "camera 0 delta time stat: (min/avg/max)" << min_ << " " << sum_ / double(counter_) << " "
                      << max_ << std::endl;
        }
      }
    }
  }
  VisionDataGlobal::singleton_.last_packets_.clear();

  return true;
}

}  // namespace vision
}  // namespace rhoban_ssl
