/*  Copyright Paul Cernea, May 2025.
All Rights Reserved.*/

#ifndef NUMBER_H
#define NUMBER_H

#include <string>

namespace FunctionalCalculator
{

/** \brief Base class from which numbers should derive. */
class Number
{
public:
  /** \brief Should implement this in order to derive from the Number class.
   *
   *  \return A complex number as a pair of double-precision real numbers a + bi;
   */
  virtual std::pair<double, double> get() const;
  /** \brief Should implement this in order to derive from the Number class. */
  virtual std::string print(bool useParentheses = false) const;
};
}

#endif //def NUMBER_H
