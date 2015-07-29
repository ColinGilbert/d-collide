################################################################################
#                                                                              #
#  Copyright (c) 2007 Ryan C. Gordon and others.                               #
#                                                                              #
#  This software is provided 'as-is', without any express or implied warranty. #
#  In no event will the authors be held liable for any damages arising from    #
#  the use of this software.                                                   #
#                                                                              #
#  Permission is granted to anyone to use this software for any purpose,       #
#  including commercial applications, and to alter it and redistribute it      #
#  freely, subject to the following restrictions:                              #
#                                                                              #
#  1. The origin of this software must not be misrepresented; you must not     #
#  claim that you wrote the original software. If you use this software in a   #
#  product, an acknowledgment in the product documentation would be            #
#  appreciated but is not required.                                            #
#                                                                              #
#  2. Altered source versions must be plainly marked as such, and must not be  #
#  misrepresented as being the original software.                              #
#                                                                              #
#  3. This notice may not be removed or altered from any source distribution.  #
#                                                                              #
#      Ryan C. Gordon <icculus@icculus.org>                                    #
#                                                                              #
################################################################################

# CMake 2.4.3 lacks a "CMake -E copy" command that handles wildcards.
#
# INPUT:
#
#   FROM - absolute pathname with wildcards to copy
#   TO - absolute pathname of directory to copy to
#
# TYPICAL USAGE, from inside a custom target or rule:
#
#   COMMAND ${CMAKE_COMMAND}
#     -D FROM=${mydirectory}/*.dll
#     -D TO=${yourdirectory}
#     -P ${CMAKE_HOME_DIRECTORY}/cp.cmake

FILE(GLOB FILELIST "${FROM}")

FOREACH(LOOPER ${FILELIST})
    MESSAGE(STATUS "Copying ${LOOPER} to ${TO}")
    EXEC_PROGRAM("${CMAKE_COMMAND}" ARGS "-E copy ${LOOPER} ${TO}"
                 OUTPUT_VARIABLE EXECOUT
                 RETURN_VALUE RC
    )
    # !!! FIXME: how do you do NOT EQUALS?
    IF(NOT RC EQUAL 0)
        MESSAGE(STATUS "${EXECOUT}")
        MESSAGE(FATAL_ERROR "Copy of '${LOOPER}' failed!")
    ENDIF(NOT RC EQUAL 0)
ENDFOREACH(LOOPER)
