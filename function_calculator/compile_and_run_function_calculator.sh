#!/bin/bash

main()
{
  local SCRIPT_DIR="$(pwd)"
  cd "${SCRIPT_DIR}"

  g++ *.cpp -o functional_calculator.o

  if [[ -a functional_calculator.o ]] ; then
    ./functional_calculator.o "${1}" "${2}" "${3}"
  fi
}

time main "${1}" "${2}" "${3}"
