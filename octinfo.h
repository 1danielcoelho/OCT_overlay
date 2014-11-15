#ifndef OCTINFO_H
#define OCTINFO_H

//Holds basic OCT parameters. Length corresponds to 'X', width to 'Y',
//depth to 'Z'
struct OCTinfo
{
  int length_steps;
  int width_steps;
  int depth_steps;

  float length_range;
  float width_range;
  float depth_range;

  float length_offset;
  float width_offset;

  //Simple comparisson operator
  bool operator==(const OCTinfo& other_struct) const;

};

//Empty default OCTinfo, used as default parameter for some functions
static OCTinfo default_oct_info = {0, 0, 0, 0, 0, 0, 0, 0};

#endif // OCTINFO_H
