

find_package(Doxygen)


if (DOXYGEN_FOUND)
    set(DOXYGEN_INPUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile.user)
    set(DOXYGEN_OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/html/index.html)
    
    set(HTML_RESOURCES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/resources)
    
    # Generate Doxygen input file from Doxygen.cmake
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.cmake ${DOXYGEN_INPUT})
    
    # Prepare output directory
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html/)
    
    # AB: taken from cmake mailinglist
    add_custom_command(
        OUTPUT ${DOXYGEN_OUTPUT}
        COMMAND ${CMAKE_COMMAND}  -E echo_append "Building API Documentation..."
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_INPUT}
        
        # copy resources to target dir
        COMMAND ${CMAKE_COMMAND}
                -D FROM=${HTML_RESOURCES_DIR}/*.gif
                -D TO=${CMAKE_CURRENT_BINARY_DIR}/html/
                -P ${CMAKE_HOME_DIRECTORY}/doc/WildcardCopy.cmake
        COMMAND ${CMAKE_COMMAND}
                -D FROM=${HTML_RESOURCES_DIR}/*.js
                -D TO=${CMAKE_CURRENT_BINARY_DIR}/html/
                -P ${CMAKE_HOME_DIRECTORY}/doc/WildcardCopy.cmake

        COMMAND ${CMAKE_COMMAND}  -E echo "Done."
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DEPENDS ${DOXYGEN_INPUT}
    )
    
    add_custom_target(apidoc ALL DEPENDS ${DOXYGEN_OUTPUT} ${HTML_RESOURCES_DIR})
    
    add_custom_target(apidoc_forced
        COMMAND ${CMAKE_COMMAND}  -E echo_append "Building API Documentation..."
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_INPUT}
        
        # copy resources to target dir
        COMMAND ${CMAKE_COMMAND}
                -D FROM=${HTML_RESOURCES_DIR}/*.gif
                -D TO=${CMAKE_CURRENT_BINARY_DIR}/html/
                -P ${CMAKE_HOME_DIRECTORY}/doc/WildcardCopy.cmake
        COMMAND ${CMAKE_COMMAND}
                -D FROM=${HTML_RESOURCES_DIR}/*.js
                -D TO=${CMAKE_CURRENT_BINARY_DIR}/html/
                -P ${CMAKE_HOME_DIRECTORY}/doc/WildcardCopy.cmake
        
        COMMAND ${CMAKE_COMMAND}  -E echo "Done."
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
    
    install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html/ DESTINATION doc/d-collide/)

endif (DOXYGEN_FOUND)

# vim: et sw=4 ts=4
