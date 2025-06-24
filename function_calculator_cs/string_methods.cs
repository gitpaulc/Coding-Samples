/*  Copyright Paul Cernea, June 2025.
All Rights Reserved.*/

namespace function_calculator_cs
{
public class StringMethods
{
  public static bool parenthesesWellFormed(in string str, in string leftRight = "()")
  {
    if (leftRight.Length != 2) { throw new System.Exception("Left-right parentheses should consist of two characters."); }
    var left = leftRight[0];
    var right = leftRight[1];
    int stackHeight = 0;
    foreach (var current in str)
    {
      if (current == left) { ++stackHeight; continue; }
      if (current == right)
      {
        if (stackHeight == 0) { return false; }
        --stackHeight;
        continue;
      }
    }
    return (stackHeight == 0);
  }

  public void trimParentheses(ref string str, in string leftRight = "()")
  {
    if (leftRight.Length != 2) { throw new System.Exception("Left-right parentheses should consist of two characters."); }
    var left = leftRight[0];
    var right = leftRight[1];
    for (int shearingString = 50; shearingString > 0; --shearingString)
    {
      if (str.Length == 0) { return; }
      if (str[0] != left) { return; }
      if (str[str.Length - 1] != right) { return; }
      var midString = str.Substring(1, str.Length - 2);
      if (!parenthesesWellFormed(midString, leftRight)) { return; }
      str = midString;
    }
  }
}
}

