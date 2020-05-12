#include <iostream>
#include <vector>

/// Two attribute class to hold each legs range data.
class leg_ranges {
  public:
  std::string name; /**< Leg Name */
  std::vector <float> range; /**< [0:2] Max XYZ, [3:5] Min XYZ. Will not work unless there are 6 points. */
  std::vector <float> target; 
  std::vector <float> offset;
};