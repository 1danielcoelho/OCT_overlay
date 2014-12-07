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

#endif // OCTINFO_H
