cmake_minimum_required(VERSION 3.21)
project(VP2S_Bozo LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)
set(VP2S_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(VP2S_BOZO_DATA "${VP2S_ROOT}/build_usa/bozo" CACHE PATH "Ignored generated game code/images")
option(VP2S_BOZO_CODE "Build the generated VP2S bring-up runner" OFF)
set(VP2S_BOZO_TEST_ISO "" CACHE FILEPATH "Optional private USA ISO for the bounded headless startup test")
set(BOZO_APP OFF CACHE BOOL "" FORCE)
set(BOZO_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(BOZO_REQUIRE_FFMPEG ON CACHE BOOL "" FORCE)
add_subdirectory("${VP2S_ROOT}/external/BozoEngine" engine)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
enable_testing()
set(FIXTURE "${CMAKE_CURRENT_BINARY_DIR}/metadata-fixture")
add_custom_command(
  OUTPUT "${FIXTURE}/fixture_000.cpp" "${FIXTURE}/fixture_table.cpp" "${FIXTURE}/fixture_functions.h"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${FIXTURE}"
  COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tests/test_bozo_prepare.py" --emit "${FIXTURE}/fixture.elf"
  COMMAND $<TARGET_FILE:bozo-recomp> "${FIXTURE}/fixture.elf" "${FIXTURE}" --prefix fixture --symbol-prefix fixture_ --files 1 --entries "${FIXTURE}/fixture.elf.entries" --no-comments
  DEPENDS bozo-recomp "${VP2S_ROOT}/tests/test_bozo_prepare.py" "${VP2S_ROOT}/tools/vp2_bozo_prepare.py")
add_executable(vp2s-bozo-metadata-check "${VP2S_ROOT}/tests/bozo_metadata_main.cpp"
  "${FIXTURE}/fixture_000.cpp" "${FIXTURE}/fixture_table.cpp")
target_include_directories(vp2s-bozo-metadata-check PRIVATE "${FIXTURE}")
target_link_libraries(vp2s-bozo-metadata-check PRIVATE bozo_runtime_core)
if(MINGW)
  target_link_options(vp2s-bozo-metadata-check PRIVATE -static)
endif()
add_test(NAME vp2s-bozo-metadata COMMAND vp2s-bozo-metadata-check)
add_test(NAME vp2s-bozo-prepare COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tests/test_bozo_prepare.py")
add_library(vp2s_bozo_kernel STATIC native_kernel.cpp native_timer.cpp native_settings.cpp native_seed.cpp native_modules.cpp native_controller.cpp)
target_link_libraries(vp2s_bozo_kernel PUBLIC bozo_port)
target_include_directories(vp2s_bozo_kernel PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
add_executable(vp2s-bozo-kernel-check "${VP2S_ROOT}/tests/bozo_kernel_main.cpp")
target_link_libraries(vp2s-bozo-kernel-check PRIVATE vp2s_bozo_kernel)
if(MINGW)
  target_link_options(vp2s-bozo-kernel-check PRIVATE -static)
endif()
add_test(NAME vp2s-bozo-kernel COMMAND vp2s-bozo-kernel-check)
add_executable(vp2s-bozo-timer-clock-check "${VP2S_ROOT}/tests/bozo_timer_clock.cpp")
target_link_libraries(vp2s-bozo-timer-clock-check PRIVATE vp2s_bozo_kernel)
if(MINGW)
  target_link_options(vp2s-bozo-timer-clock-check PRIVATE -static)
endif()
add_test(NAME vp2s-bozo-timer-clock COMMAND vp2s-bozo-timer-clock-check)
add_executable(vp2s-bozo-settings-check "${VP2S_ROOT}/tests/bozo_settings_main.cpp")
target_link_libraries(vp2s-bozo-settings-check PRIVATE vp2s_bozo_kernel)
if(MINGW)
  target_link_options(vp2s-bozo-settings-check PRIVATE -static)
endif()
add_test(NAME vp2s-bozo-settings COMMAND vp2s-bozo-settings-check)
add_executable(vp2s-bozo-seed-check "${VP2S_ROOT}/tests/bozo_seed_main.cpp")
target_link_libraries(vp2s-bozo-seed-check PRIVATE vp2s_bozo_kernel)
if(MINGW)
  target_link_options(vp2s-bozo-seed-check PRIVATE -static)
endif()
add_test(NAME vp2s-bozo-seed COMMAND vp2s-bozo-seed-check)
add_executable(vp2s-bozo-modules-check "${VP2S_ROOT}/tests/bozo_modules_main.cpp")
target_link_libraries(vp2s-bozo-modules-check PRIVATE vp2s_bozo_kernel)
if(MINGW)
  target_link_options(vp2s-bozo-modules-check PRIVATE -static)
endif()
add_test(NAME vp2s-bozo-modules COMMAND vp2s-bozo-modules-check)
if(VP2S_BOZO_CODE)
  set(CONTROLLER_REF "${CMAKE_CURRENT_BINARY_DIR}/controller-reference")
  add_custom_command(OUTPUT "${CONTROLLER_REF}/ds2o_d_iop.cpp" "${CONTROLLER_REF}/ds2.bin" "${CONTROLLER_REF}/vp2_controller_image.h"
    COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_controller_fixture.py"
      "${VP2S_BOZO_DATA}/../extract/iop/ds2o_d.irx" "${CONTROLLER_REF}" --build "${VP2S_BOZO_DATA}/.."
    DEPENDS "${VP2S_ROOT}/tools/vp2_bozo_controller_fixture.py" "${VP2S_BOZO_DATA}/../extract/iop/ds2o_d.irx"
      "${VP2S_ROOT}/external/BozoEngine/tools/native_irx/relocate.py" "${VP2S_ROOT}/external/BozoEngine/tools/native_irx/recompile.py")
  add_executable(vp2s-bozo-controller-check "${VP2S_ROOT}/tests/bozo_controller_main.cpp"
    native_controller.cpp "${CONTROLLER_REF}/ds2o_d_iop.cpp")
  target_include_directories(vp2s-bozo-controller-check PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(vp2s-bozo-controller-check PRIVATE bozo_libs)
  if(MINGW)
    target_link_options(vp2s-bozo-controller-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-controller-records COMMAND vp2s-bozo-controller-check "${CONTROLLER_REF}/ds2.bin")
  set_tests_properties(vp2s-bozo-controller-records PROPERTIES TIMEOUT 10)
  set(DBC_REF "${CMAKE_CURRENT_BINARY_DIR}/dbc-native-reference")
  add_custom_command(
    OUTPUT "${DBC_REF}/dbc_manager_iop.cpp" "${DBC_REF}/vp2_dbc_image.h"
    COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_dbc_fixture.py"
      "${VP2S_BOZO_DATA}/../extract/iop/Dbc_Manager.irx" "${DBC_REF}" --build "${VP2S_BOZO_DATA}/.."
    DEPENDS "${VP2S_ROOT}/tools/vp2_bozo_dbc_fixture.py" "${VP2S_BOZO_DATA}/../extract/iop/Dbc_Manager.irx"
      "${VP2S_ROOT}/external/BozoEngine/tools/native_irx/relocate.py" "${VP2S_ROOT}/external/BozoEngine/tools/native_irx/recompile.py")
  add_executable(vp2s-bozo-dbc-check "${VP2S_ROOT}/tests/bozo_dbc_main.cpp" "${DBC_REF}/dbc_manager_iop.cpp")
  target_include_directories(vp2s-bozo-dbc-check PRIVATE "${DBC_REF}")
  target_link_libraries(vp2s-bozo-dbc-check PRIVATE bozo_libs)
  if(MINGW)
    target_link_options(vp2s-bozo-dbc-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-dbc-init COMMAND vp2s-bozo-dbc-check)
  set_tests_properties(vp2s-bozo-dbc-init PROPERTIES TIMEOUT 10)
  add_executable(vp2s-bozo-controller-bridge-check "${VP2S_ROOT}/tests/bozo_controller_bridge.cpp"
    native_controller.cpp "${CONTROLLER_REF}/ds2o_d_iop.cpp" "${DBC_REF}/dbc_manager_iop.cpp")
  target_include_directories(vp2s-bozo-controller-bridge-check PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}" "${CONTROLLER_REF}" "${DBC_REF}")
  target_link_libraries(vp2s-bozo-controller-bridge-check PRIVATE bozo_libs)
  if(MINGW)
    target_link_options(vp2s-bozo-controller-bridge-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-controller-bridge COMMAND vp2s-bozo-controller-bridge-check --connected)
  set_tests_properties(vp2s-bozo-controller-bridge PROPERTIES TIMEOUT 10)
  add_test(NAME vp2s-bozo-module-identities COMMAND vp2s-bozo-modules-check "${VP2S_BOZO_DATA}/../extract/iop")
  set(RECORD_REF "${CMAKE_CURRENT_BINARY_DIR}/timer-record-reference")
  add_custom_command(
    OUTPUT "${RECORD_REF}/timer_record_000.cpp" "${RECORD_REF}/timer_record_table.cpp" "${RECORD_REF}/timer_record_functions.h"
    COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_timer_record_fixture.py" "${VP2S_BOZO_DATA}/images/main.elf" "${RECORD_REF}" --build "${VP2S_BOZO_DATA}/.."
    COMMAND $<TARGET_FILE:bozo-recomp> "${RECORD_REF}/fixture.elf" "${RECORD_REF}" --prefix timer_record --symbol-prefix timer_record_
      --files 1 --externals "${RECORD_REF}/externals.txt" --no-comments
    DEPENDS bozo-recomp "${VP2S_BOZO_DATA}/images/main.elf" "${VP2S_ROOT}/tools/vp2_bozo_timer_record_fixture.py"
      "${VP2S_ROOT}/tools/vp2_bozo_kernel.py" "${VP2S_ROOT}/tools/vp2_bozo_prepare.py")
  add_executable(vp2s-bozo-timer-callback-check "${VP2S_ROOT}/tests/bozo_timer_records.cpp"
    "${RECORD_REF}/timer_record_000.cpp" "${RECORD_REF}/timer_record_table.cpp")
  target_include_directories(vp2s-bozo-timer-callback-check PRIVATE "${RECORD_REF}")
  target_link_libraries(vp2s-bozo-timer-callback-check PRIVATE vp2s_bozo_kernel)
  if(MINGW)
    target_link_options(vp2s-bozo-timer-callback-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-timer-callback-records COMMAND vp2s-bozo-timer-callback-check)
  set(TIMING_REF "${CMAKE_CURRENT_BINARY_DIR}/timing-reference")
  foreach(VARIANT original adapted)
    set(TIMING_DIR "${TIMING_REF}/${VARIANT}")
    set(TIMING_PATCH_ARGS)
    if(VARIANT STREQUAL "adapted")
      set(TIMING_PATCH_ARGS --patches "${TIMING_DIR}/patches.txt")
    endif()
    add_custom_command(
      OUTPUT "${TIMING_DIR}/timing_${VARIANT}_000.cpp" "${TIMING_DIR}/timing_${VARIANT}_table.cpp" "${TIMING_DIR}/timing_${VARIANT}_functions.h"
      COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_timing_fixture.py" "${VP2S_BOZO_DATA}/images/main.elf" "${TIMING_DIR}" --build "${VP2S_BOZO_DATA}/.."
      COMMAND $<TARGET_FILE:bozo-recomp> "${TIMING_DIR}/fixture.elf" "${TIMING_DIR}" --prefix timing_${VARIANT}
        --symbol-prefix timing_${VARIANT}_ --files 1 --externals "${TIMING_DIR}/externals.txt" ${TIMING_PATCH_ARGS} --no-comments
      DEPENDS bozo-recomp "${VP2S_BOZO_DATA}/images/main.elf" "${VP2S_ROOT}/tools/vp2_bozo_timing_fixture.py"
        "${VP2S_ROOT}/tools/vp2_bozo_kernel.py" "${VP2S_ROOT}/tools/vp2_bozo_prepare.py")
    list(APPEND TIMING_SOURCES "${TIMING_DIR}/timing_${VARIANT}_000.cpp" "${TIMING_DIR}/timing_${VARIANT}_table.cpp")
  endforeach()
  add_executable(vp2s-bozo-timing-check "${VP2S_ROOT}/tests/bozo_timing_main.cpp" ${TIMING_SOURCES})
  target_include_directories(vp2s-bozo-timing-check PRIVATE "${TIMING_REF}/original" "${TIMING_REF}/adapted")
  target_link_libraries(vp2s-bozo-timing-check PRIVATE bozo_runtime_core)
  if(MINGW)
    target_link_options(vp2s-bozo-timing-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-timing-prefix COMMAND vp2s-bozo-timing-check)
  set(TIMER_REF "${CMAKE_CURRENT_BINARY_DIR}/timer-reference")
  add_custom_command(
    OUTPUT "${TIMER_REF}/timer_ref_000.cpp" "${TIMER_REF}/timer_ref_table.cpp" "${TIMER_REF}/timer_ref_functions.h"
    COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_timer_fixture.py" "${VP2S_BOZO_DATA}/images/main.elf" "${TIMER_REF}" --build "${VP2S_BOZO_DATA}/.."
    COMMAND $<TARGET_FILE:bozo-recomp> "${TIMER_REF}/fixture.elf" "${TIMER_REF}" --prefix timer_ref --symbol-prefix timer_original_
      --files 1 --externals "${TIMER_REF}/externals.txt" --no-comments
    DEPENDS bozo-recomp "${VP2S_BOZO_DATA}/images/main.elf" "${VP2S_ROOT}/tools/vp2_bozo_timer_fixture.py"
      "${VP2S_ROOT}/tools/vp2_bozo_kernel.py" "${VP2S_ROOT}/tools/vp2_bozo_prepare.py")
  add_executable(vp2s-bozo-timer-check "${VP2S_ROOT}/tests/bozo_timer_main.cpp"
    "${TIMER_REF}/timer_ref_000.cpp" "${TIMER_REF}/timer_ref_table.cpp")
  target_include_directories(vp2s-bozo-timer-check PRIVATE "${TIMER_REF}")
  target_link_libraries(vp2s-bozo-timer-check PRIVATE vp2s_bozo_kernel)
  if(MINGW)
    target_link_options(vp2s-bozo-timer-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-timer-records COMMAND vp2s-bozo-timer-check)
  add_test(NAME vp2s-bozo-kernel-signatures COMMAND ${Python3_EXECUTABLE}
    "${VP2S_ROOT}/tests/test_bozo_kernel.py" "${VP2S_BOZO_DATA}/images/main.elf")
  include("${VP2S_BOZO_DATA}/sources.cmake")
  add_library(vp2s_bozo_game STATIC ${VP2S_BOZO_SOURCES})
  target_link_libraries(vp2s_bozo_game PUBLIC bozo_runtime_headers)
  target_include_directories(vp2s_bozo_game PUBLIC "${VP2S_BOZO_DATA}/generated")
  target_compile_options(vp2s_bozo_game PRIVATE -O1 -w -ffp-contract=off)
  add_executable(vp2s-bozo-headless main.cpp "${DBC_REF}/dbc_manager_iop.cpp" "${CONTROLLER_REF}/ds2o_d_iop.cpp")
  target_include_directories(vp2s-bozo-headless PRIVATE "${DBC_REF}" "${CONTROLLER_REF}")
  target_link_libraries(vp2s-bozo-headless PRIVATE vp2s_bozo_game vp2s_bozo_kernel bozo_host)
  target_compile_definitions(vp2s-bozo-headless PRIVATE VP2S_BOZO_IMAGES="${VP2S_BOZO_DATA}/images")
  if(MINGW)
    target_link_options(vp2s-bozo-headless PRIVATE -static)
  endif()
  if(VP2S_BOZO_TEST_ISO)
    add_test(NAME vp2s-bozo-startup COMMAND ${Python3_EXECUTABLE}
      "${VP2S_ROOT}/tests/test_bozo_bootstrap.py" $<TARGET_FILE:vp2s-bozo-headless> "${VP2S_BOZO_TEST_ISO}")
    set_tests_properties(vp2s-bozo-startup PROPERTIES TIMEOUT 40)
  endif()
endif()
