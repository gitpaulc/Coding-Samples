#!/bin/bash

main()
{
	local SCRIPT_DIR="$(pwd)"
	cd "${SCRIPT_DIR}"

	# Adding -w flag to suppress all warnings because of GLUT deprecation warnings for Mac OSX.
	g++ *.cpp -o graphing_math.o -framework OpenGL -framework GLUT -w

	if [[ -a graphing_math.o ]] ; then
		./graphing_math.o "${1}" "${2}"
	fi
}

time main "${1}" "${2}"
