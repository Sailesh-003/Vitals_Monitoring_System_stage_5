# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "C:/ncs/v2.9.1/nrf/samples/Adpd144ri")
  file(MAKE_DIRECTORY "C:/ncs/v2.9.1/nrf/samples/Adpd144ri")
endif()
file(MAKE_DIRECTORY
  "C:/ncs/v2.9.1/nrf/samples/Adpd144ri/build/Adpd144ri"
  "C:/ncs/v2.9.1/nrf/samples/Adpd144ri/build/_sysbuild/sysbuild/images/Adpd144ri-prefix"
  "C:/ncs/v2.9.1/nrf/samples/Adpd144ri/build/_sysbuild/sysbuild/images/Adpd144ri-prefix/tmp"
  "C:/ncs/v2.9.1/nrf/samples/Adpd144ri/build/_sysbuild/sysbuild/images/Adpd144ri-prefix/src/Adpd144ri-stamp"
  "C:/ncs/v2.9.1/nrf/samples/Adpd144ri/build/_sysbuild/sysbuild/images/Adpd144ri-prefix/src"
  "C:/ncs/v2.9.1/nrf/samples/Adpd144ri/build/_sysbuild/sysbuild/images/Adpd144ri-prefix/src/Adpd144ri-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/ncs/v2.9.1/nrf/samples/Adpd144ri/build/_sysbuild/sysbuild/images/Adpd144ri-prefix/src/Adpd144ri-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/ncs/v2.9.1/nrf/samples/Adpd144ri/build/_sysbuild/sysbuild/images/Adpd144ri-prefix/src/Adpd144ri-stamp${cfgdir}") # cfgdir has leading slash
endif()
