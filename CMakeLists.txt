CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
project(IMSI-catcher)
message( STATUS "CMAKE_SYSTEM: " ${CMAKE_SYSTEM} )
message( STATUS "CMAKE_SYSTEM_PROCESSOR: " ${CMAKE_SYSTEM_PROCESSOR} )
message( STATUS "CMAKE_CXX_COMPILER: " ${CMAKE_CXX_COMPILER} )
message(${CMAKE_BINARY_DIR})

option(ENABLE_SRSUE    "Build srsUE application"                  ON)
option(ENABLE_SRSENB   "Build srsENB application"                 ON)
option(ENABLE_SRSEPC   "Build srsEPC application"                 ON)
option(ENABLE_5GNR     "Build with 5G-NR components"              OFF)
option(DISABLE_SIMD    "Disable SIMD instructions"                OFF)
option(AUTO_DETECT_ISA "Autodetect supported ISA extensions"      ON)

option(ENABLE_GUI      "Enable GUI (using srsGUI)"                ON)
option(ENABLE_UHD      "Enable UHD"                               ON)
option(ENABLE_BLADERF  "Enable BladeRF"                           OFF)
option(ENABLE_SOAPYSDR "Enable SoapySDR"                          OFF)
option(ENABLE_ZEROMQ   "Enable ZeroMQ"                            ON)
option(ENABLE_HARDSIM  "Enable support for SIM cards"             ON)

option(ENABLE_TTCN3    "Enable TTCN3 test binaries"               OFF)
option(ENABLE_ZMQ_TEST "Enable ZMQ based E2E tests"               OFF)

option(BUILD_STATIC    "Attempt to statically link external deps" OFF)
option(RPATH           "Enable RPATH"                             OFF)
option(ENABLE_ASAN     "Enable gcc/clang address sanitizer"       OFF)
option(ENABLE_GCOV     "Enable gcc/clang address sanitizer"       OFF)
option(ENABLE_MSAN     "Enable clang memory sanitizer"            OFF)
option(ENABLE_TIDY     "Enable clang tidy"                        OFF)

option(USE_LTE_RATES   "Use standard LTE sampling rates"          OFF)
option(USE_MKL         "Use MKL instead of fftw"                  OFF)

option(ENABLE_TIMEPROF "Enable time profiling"                    ON)

option(FORCE_32BIT     "Add flags to force 32 bit compilation"    OFF)

include_directories(${PROJECT_BINARY_DIR}/srsLTE/lib/include)
include_directories(${PROJECT_SOURCE_DIR}/srsLTE/lib/include)
include_directories(${PROJECT_SOURCE_DIR}/srsLTE)
add_subdirectory(srsLTE)
add_subdirectory(src)
