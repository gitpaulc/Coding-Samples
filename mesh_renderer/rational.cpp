
#include "rational.h"

#include <stdexcept>

namespace ComputationalGeometry
{
  Rational::Rational(int nn, int dd)
  {
    if (dd == 0)
    {
      throw std::invalid_argument("Division by zero.");
    }
    int gcd_ = gcd(nn, dd);
    num = nn; denom = dd;
    if (gcd_ != 0)
    {
      num = num / gcd_;
      denom = denom / gcd_;
    }
    if (denom < 0)
    {
      num *= -1;
      denom *= -1;
    }
  }

  Rational::Rational(const Rational& rhs)
  {
    num = rhs.num;
    denom = rhs.denom;
  }

  Rational::Rational(Rational&& rhs) noexcept
  {
    num = rhs.num;
    denom = rhs.denom;
  }

  Rational& Rational::operator=(const Rational& rhs)
  {
    if (this != &rhs)
    {
      num = rhs.num;
      denom = rhs.denom;
    }
    return *this;
  }

  Rational& Rational::operator=(Rational&& rhs) noexcept
  {
    if (this != &rhs)
    {
      num = rhs.num;
      denom = rhs.denom;
    }
    return *this;
  }

  int Rational::gcd(int aa, int bb)
  {
    if ((aa == bb) || (bb == 0)) { return (aa > 0) ? aa : (-aa); }
    if (aa == 0) { return (bb > 0) ? bb : (-bb); }
    int abs_a = (aa > 0) ? aa : -aa;
    int abs_b = (bb > 0) ? bb : -bb;
    //if (bb != 0) { return gcd(bb, aa % bb); }
    for (int safety_counter = 2 * abs_a + 2 * abs_b; bb != 0; --safety_counter)
    {
      if (safety_counter <= 0) { break; }
      int aa_old = aa;
      int bb_old = bb;
      aa = bb_old;
      bb = aa_old % bb_old;
    }
    if (aa < 0) { return -aa; }
    return aa;
  }

  Rational Rational::operator+() const
  {
    return *this;
  }

  Rational Rational::operator-() const
  {
    return Rational(-num, denom);
  }

  Rational Rational::operator+(const Rational& rhs) const
  {
    return Rational(num * rhs.denom + rhs.num * denom, denom * rhs.denom);
  }

  Rational Rational::operator-(const Rational& rhs) const
  {
    return ((*this) + (-rhs));
  }

  Rational Rational::operator*(const Rational& rhs) const
  {
    return Rational(num * rhs.num, denom * rhs.denom);
  }

  Rational Rational::operator/(const Rational& rhs) const
  {
    if (rhs.num == 0)
    {
      throw std::invalid_argument("Operator division by zero.");
    }
    return Rational(num * rhs.denom, denom * rhs.num);
  }

  bool Rational::operator==(const Rational& rhs) const
  {
    if (rhs.num != num) { return false; }
    if (rhs.denom != denom) { return false; }
    return true;
  }

  bool Rational::operator!=(const Rational& rhs) const
  {
    if (*this == rhs) { return false; }
    return true;
  }

  bool Rational::operator<(const Rational& rhs) const
  {
    if (num * rhs.denom < denom * rhs.num) { return true; }
    return false;
  }

  bool Rational::operator>(const Rational& rhs) const
  {
    if (num * rhs.denom > denom * rhs.num) { return true; }
    return false;
  }

  bool Rational::operator<=(const Rational& rhs) const
  {
    if (num * rhs.denom <= denom * rhs.num) { return true; }
    return false;
  }

  bool Rational::operator>=(const Rational& rhs) const
  {
    if (num * rhs.denom >= denom * rhs.num) { return true; }
    return false;
  }

  double Rational::get() const
  {
    double nn = (double)num;
    double dd = (double)denom;
    return nn / dd;
  }
}
