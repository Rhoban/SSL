#include "time_synchronizer.h"

#include <rhoban_utils/timing/time_stamp.h>
#include <debug.h>

namespace rhoban_ssl
{
TimeSynchronizer::TimeSynchronizer() : diff_buffer_(BUFFER_SIZE), shift_buffer_(BUFFER_SIZE), computed_time_shift(0)
{
}

void TimeSynchronizer::update(const vision::CameraDetectionFrame& detection_frame)
{
  double now = rhoban_utils::TimeStamp::now().getTimeSec();
  double local_t_sent = detection_frame.t_sent_ + computed_time_shift;
  double diff = now - local_t_sent;

  diff_buffer_.insert(diff);

  double average_diff = std::abs(average(diff_buffer_));

  bool not_synchronised = average_diff > 3e-1;
  bool synchronisation_in_progress = shift_buffer_.numberOfElements() > 0;
  if (not_synchronised || synchronisation_in_progress)
  {
    double new_time_shift = now - detection_frame.t_sent_;
    shift_buffer_.insert(new_time_shift);

    computed_time_shift = average(shift_buffer_);
    bool synchronised = average_diff < 1e-4;  // precision 0.1ms
    if (synchronised)
    {
      DEBUG("Synced with vision clock.");
      shift_buffer_.clear();
    }
  }
}

void TimeSynchronizer::syncTimeShift(double* time_shift_with_vision)
{
  *time_shift_with_vision = computed_time_shift;
}

double TimeSynchronizer::average(const CircularVector<double>& buffer)
{
  assert(buffer.size() > 0);

  uint size = buffer.size();
  double avg = 0.0;
  for (uint i = 0; i < size; ++i)
  {
    avg += buffer[i];
  }
  return avg / size;
}

}  // namespace rhoban_ssl
