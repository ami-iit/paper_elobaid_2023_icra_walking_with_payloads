# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.


if(BUILD_TESTING)

  configure_file(cmake/Catch2Main.cpp.in ${CMAKE_BINARY_DIR}/Testing/Catch2Main.cpp)
  add_library(CatchTestMain ${CMAKE_BINARY_DIR}/Testing/Catch2Main.cpp)
  target_link_libraries(CatchTestMain PUBLIC Catch2::Catch2)

endif()

function(add_centroidal_mpc_walking_test)

  if(BUILD_TESTING)

    set(options )
    set(oneValueArgs NAME)
    set(multiValueArgs SOURCES LINKS)

    set(prefix "centroidal")

    cmake_parse_arguments(${prefix}
      "${options}"
      "${oneValueArgs}"
      "${multiValueArgs}"
      ${ARGN})

    set(name ${${prefix}_NAME})
    set(unit_test_files ${${prefix}_SOURCES})

    set(targetname ${name}UnitTests)
    add_executable(${targetname}
      "${unit_test_files}")

    target_link_libraries(${targetname} PRIVATE CatchTestMain ${${prefix}_LINKS})
    target_compile_definitions(${targetname} PRIVATE CATCH_CONFIG_FAST_COMPILE CATCH_CONFIG_DISABLE_MATCHERS)
    target_compile_features(${targetname} PUBLIC cxx_std_17)
    target_compile_definitions(${targetname} PRIVATE -D_USE_MATH_DEFINES)

    add_test(NAME ${targetname} COMMAND ${targetname})

  endif()

endfunction()
