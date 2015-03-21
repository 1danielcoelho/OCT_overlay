#include "octinfo.h"

bool OCTinfo::operator==(const OCTinfo& other_struct) const {
  return length_steps == other_struct.length_steps &&
         width_steps == other_struct.width_steps &&
         depth_steps == other_struct.depth_steps &&
         length_range == other_struct.depth_range &&
         width_range == other_struct.width_range &&
         depth_range == other_struct.depth_range &&
         length_offset == other_struct.length_offset &&
         width_offset == other_struct.width_offset;
}
