/*  Copyright Paul Cernea, September 2024.
All Rights Reserved.*/

#ifndef EDGE_LIST_H
#define EDGE_LIST_H

#include "includes.h"

namespace ComputationalGeometry
{
  class DoublyConnectedEdgeList
  {
    class Impl;
    std::unique_ptr<Impl> pImpl;
    public:
      DoublyConnectedEdgeList();
  };
}

#endif //def EDGE_LIST
