/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#ifndef FUNCTION_H
#define FUNCTION_H

#include <string>

namespace FunctionalCalculator
{

/** \brief Base abstract class from which functions should derive. */
class Function
{
public:
  /** \brief Implement this in order to derive from the Function class. */
  virtual double eval(double x, double y, double z) const = 0;
};
}

#endif //def FUNCTION_H
