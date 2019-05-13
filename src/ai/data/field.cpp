#include "field.h"

namespace rhoban_ssl
{
namespace data
{
Field::Field()
{
  field_length_ = 9.0;
  field_width_ = 6.0;
  goal_width_ = 1.0;
  goal_depth_ = 0.6;
  boundary_width_ = 0.1;
  penalty_area_depth_ = 1.0;
  penalty_area_width_ = 2.0;
}

}  // namespace data
}  // namespace rhoban_ssl
