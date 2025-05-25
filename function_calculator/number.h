/*  Copyright Paul Cernea, August 2024.
All Rights Reserved.*/

#ifndef NUMBER_H
#define NUMBER_H

#include <string>

namespace FunctionalCalculator
{

/** \brief Base abstract class from which numbers should derive. */
class Number
{
public:
  /** \brief Implement this in order to derive from the Number class. */
  virtual std::string print() const = 0;
};
}

#endif //def NUMBER
