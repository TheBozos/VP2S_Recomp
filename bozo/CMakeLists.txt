cmake_minimum_required(VERSION 3.21)
project(VP2S_Bozo LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)
set(VP2S_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
set(VP2S_BOZO_DATA "${VP2S_ROOT}/build_usa/bozo" CACHE PATH "Ignored generated game code/images")
option(VP2S_BOZO_CODE "Build the generated VP2S bring-up runner" OFF)
option(VP2S_BOZO_GPU_TESTS "Run synthetic renderer readbacks in a hidden window" OFF)
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
add_test(NAME vp2s-bozo-texture-manifest COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tests/test_bozo_texture_manifest.py")
add_test(NAME vp2s-bozo-glyph-conversion COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tests/test_bozo_glyphs.py")
add_library(vp2s_bozo_kernel STATIC native_kernel.cpp native_timer.cpp native_settings.cpp native_seed.cpp native_modules.cpp native_controller.cpp native_frame.cpp native_bootstrap.cpp native_random.cpp native_math.cpp native_object_math.cpp native_trig.cpp native_frustum.cpp native_textures.cpp native_scene.cpp native_assets.cpp native_debug_font.cpp native_state_setup.cpp native_screen_rect.cpp native_glyphs.cpp native_ui_text.cpp)
set(GLYPH_FIXTURE "${CMAKE_CURRENT_BINARY_DIR}/glyphs-fixture")
add_custom_command(OUTPUT "${GLYPH_FIXTURE}/manifest.txt"
  COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tests/test_bozo_glyphs.py" --emit "${GLYPH_FIXTURE}"
  DEPENDS "${VP2S_ROOT}/tests/test_bozo_glyphs.py" "${VP2S_ROOT}/tools/vp2_glyphs.py")
add_custom_target(vp2s-bozo-glyph-fixture DEPENDS "${GLYPH_FIXTURE}/manifest.txt")
add_executable(vp2s-bozo-glyph-check "${VP2S_ROOT}/tests/bozo_glyphs_main.cpp")
add_dependencies(vp2s-bozo-glyph-check vp2s-bozo-glyph-fixture)
target_link_libraries(vp2s-bozo-glyph-check PRIVATE vp2s_bozo_kernel)
if(MINGW)
  target_link_options(vp2s-bozo-glyph-check PRIVATE -static)
endif()
add_test(NAME vp2s-bozo-glyph-assets COMMAND vp2s-bozo-glyph-check "${GLYPH_FIXTURE}")
if(VP2S_BOZO_GPU_TESTS)
  find_package(SDL3 CONFIG REQUIRED)
  add_executable(vp2s-bozo-texture-alpha-gpu "${VP2S_ROOT}/tests/bozo_texture_alpha_gpu_main.cpp")
  target_link_libraries(vp2s-bozo-texture-alpha-gpu PRIVATE vp2s_bozo_kernel bozo_render SDL3::SDL3-static)
  if(MINGW)
    target_link_options(vp2s-bozo-texture-alpha-gpu PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-texture-alpha-gpu COMMAND vp2s-bozo-texture-alpha-gpu)
  set_tests_properties(vp2s-bozo-texture-alpha-gpu PROPERTIES LABELS "gpu")
endif()
if(MSVC)
  set_source_files_properties(native_trig.cpp PROPERTIES COMPILE_OPTIONS "/fp:strict")
  set_source_files_properties(native_frustum.cpp PROPERTIES COMPILE_OPTIONS "/arch:AVX2;/fp:strict")
else()
  set_source_files_properties(native_trig.cpp PROPERTIES COMPILE_OPTIONS "-ffp-contract=off")
  set_source_files_properties(native_frustum.cpp PROPERTIES COMPILE_OPTIONS "-mavx2;-ffp-contract=off")
endif()
add_executable(vp2s-bozo-assets-check "${VP2S_ROOT}/tests/bozo_assets_main.cpp")
target_link_libraries(vp2s-bozo-assets-check PRIVATE vp2s_bozo_kernel)
if(MINGW)
  target_link_options(vp2s-bozo-assets-check PRIVATE -static)
endif()
add_test(NAME vp2s-bozo-asset-provenance COMMAND vp2s-bozo-assets-check)
if(MSVC)
  set_source_files_properties(native_bootstrap.cpp native_math.cpp native_object_math.cpp PROPERTIES COMPILE_OPTIONS "/arch:AVX2")
else()
  set_source_files_properties(native_bootstrap.cpp native_math.cpp native_object_math.cpp PROPERTIES COMPILE_OPTIONS "-mavx2")
endif()
add_executable(vp2s-bozo-replies-check "${VP2S_ROOT}/tests/bozo_replies_main.cpp")
target_link_libraries(vp2s-bozo-replies-check PRIVATE vp2s_bozo_kernel)
if(MINGW)
  target_link_options(vp2s-bozo-replies-check PRIVATE -static)
endif()
add_test(NAME vp2s-bozo-command-replies COMMAND vp2s-bozo-replies-check)
add_executable(vp2s-bozo-bootstrap-check "${VP2S_ROOT}/tests/bozo_bootstrap_main.cpp")
target_link_libraries(vp2s-bozo-bootstrap-check PRIVATE vp2s_bozo_kernel)
if(MINGW)
  target_link_options(vp2s-bozo-bootstrap-check PRIVATE -static)
endif()
add_test(NAME vp2s-bozo-bootstrap-key COMMAND vp2s-bozo-bootstrap-check
  "${VP2S_BOZO_DATA}/images/bootstrap.bin" "${VP2S_ROOT}/build_usa/native_bootstrap/live.bin")
add_executable(vp2s-bozo-random-check "${VP2S_ROOT}/tests/bozo_random_main.cpp")
target_link_libraries(vp2s-bozo-random-check PRIVATE vp2s_bozo_kernel)
if(MINGW)
  target_link_options(vp2s-bozo-random-check PRIVATE -static)
endif()
add_test(NAME vp2s-bozo-procedural-random COMMAND vp2s-bozo-random-check)
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
  set(DISPLAY_REF "${CMAKE_CURRENT_BINARY_DIR}/display-reference")
  foreach(VARIANT original adapted)
    set(DISPLAY_DIR "${DISPLAY_REF}/${VARIANT}")
    set(DISPLAY_PATCH_ARGS)
    if(VARIANT STREQUAL "adapted")
      set(DISPLAY_PATCH_ARGS --patches "${DISPLAY_DIR}/patches.txt")
    endif()
    add_custom_command(OUTPUT "${DISPLAY_DIR}/display_${VARIANT}_000.cpp" "${DISPLAY_DIR}/display_${VARIANT}_table.cpp" "${DISPLAY_DIR}/display_${VARIANT}_functions.h"
      COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_display_fixture.py" "${VP2S_BOZO_DATA}/images/main.elf" "${DISPLAY_DIR}" --build "${VP2S_BOZO_DATA}/.." --variant ${VARIANT}
      COMMAND $<TARGET_FILE:bozo-recomp> "${DISPLAY_DIR}/fixture.elf" "${DISPLAY_DIR}" --prefix display_${VARIANT} --symbol-prefix display_${VARIANT}_
        --files 1 --externals "${DISPLAY_DIR}/externals.txt" ${DISPLAY_PATCH_ARGS} --no-comments
      DEPENDS bozo-recomp "${VP2S_ROOT}/tools/vp2_bozo_display_fixture.py" "${VP2S_ROOT}/tools/vp2_bozo_kernel.py" "${VP2S_BOZO_DATA}/images/main.elf")
    list(APPEND DISPLAY_SOURCES "${DISPLAY_DIR}/display_${VARIANT}_000.cpp" "${DISPLAY_DIR}/display_${VARIANT}_table.cpp")
  endforeach()
  add_executable(vp2s-bozo-display-check "${VP2S_ROOT}/tests/bozo_display_main.cpp" ${DISPLAY_SOURCES})
  target_include_directories(vp2s-bozo-display-check PRIVATE "${DISPLAY_REF}/original" "${DISPLAY_REF}/adapted")
  target_link_libraries(vp2s-bozo-display-check PRIVATE bozo_port)
  if(MINGW)
    target_link_options(vp2s-bozo-display-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-display-selection COMMAND vp2s-bozo-display-check)
  set(MENU_CLEAR_REF "${CMAKE_CURRENT_BINARY_DIR}/menu-clear-reference")
  foreach(VARIANT original adapted)
    set(MENU_CLEAR_DIR "${MENU_CLEAR_REF}/${VARIANT}")
    set(MENU_CLEAR_PATCH_ARGS)
    if(VARIANT STREQUAL "adapted")
      set(MENU_CLEAR_PATCH_ARGS --patches "${MENU_CLEAR_DIR}/patches.txt")
    endif()
    add_custom_command(OUTPUT "${MENU_CLEAR_DIR}/menu_clear_${VARIANT}_000.cpp" "${MENU_CLEAR_DIR}/menu_clear_${VARIANT}_table.cpp" "${MENU_CLEAR_DIR}/menu_clear_${VARIANT}_functions.h"
      COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_menu_clear_fixture.py" "${VP2S_BOZO_DATA}/images/mod002.elf" "${MENU_CLEAR_DIR}" --build "${VP2S_BOZO_DATA}/.."
      COMMAND $<TARGET_FILE:bozo-recomp> "${MENU_CLEAR_DIR}/fixture.elf" "${MENU_CLEAR_DIR}" --prefix menu_clear_${VARIANT} --symbol-prefix menu_clear_${VARIANT}_
        --files 1 --externals "${MENU_CLEAR_DIR}/externals.txt" ${MENU_CLEAR_PATCH_ARGS} --no-comments
      DEPENDS bozo-recomp "${VP2S_ROOT}/tools/vp2_bozo_menu_clear_fixture.py" "${VP2S_ROOT}/tools/vp2_bozo_kernel.py" "${VP2S_BOZO_DATA}/images/mod002.elf")
    list(APPEND MENU_CLEAR_SOURCES "${MENU_CLEAR_DIR}/menu_clear_${VARIANT}_000.cpp" "${MENU_CLEAR_DIR}/menu_clear_${VARIANT}_table.cpp")
  endforeach()
  add_executable(vp2s-bozo-menu-clear-check "${VP2S_ROOT}/tests/bozo_menu_clear_main.cpp" ${MENU_CLEAR_SOURCES})
  target_include_directories(vp2s-bozo-menu-clear-check PRIVATE "${MENU_CLEAR_REF}/original" "${MENU_CLEAR_REF}/adapted")
  target_link_libraries(vp2s-bozo-menu-clear-check PRIVATE bozo_runtime_core)
  if(MINGW)
    target_link_options(vp2s-bozo-menu-clear-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-menu-clear COMMAND vp2s-bozo-menu-clear-check)
  set(TEXTURE_REF "${CMAKE_CURRENT_BINARY_DIR}/texture-reference")
  set(UI_TEXT_REF "${CMAKE_CURRENT_BINARY_DIR}/ui-text-reference")
  foreach(VARIANT original adapted)
    set(UI_TEXT_DIR "${UI_TEXT_REF}/${VARIANT}")
    add_custom_command(OUTPUT "${UI_TEXT_DIR}/ui_text_${VARIANT}_000.cpp" "${UI_TEXT_DIR}/ui_text_${VARIANT}_table.cpp" "${UI_TEXT_DIR}/ui_text_${VARIANT}_functions.h"
      COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_ui_text_fixture.py" "${VP2S_BOZO_DATA}/images/mod001.elf" "${UI_TEXT_DIR}" --build "${VP2S_BOZO_DATA}/.." --variant ${VARIANT}
      COMMAND $<TARGET_FILE:bozo-recomp> "${UI_TEXT_DIR}/fixture.elf" "${UI_TEXT_DIR}" --prefix ui_text_${VARIANT} --symbol-prefix ui_text_${VARIANT}_
        --files 1 --externals "${UI_TEXT_DIR}/externals.txt" --no-comments
      DEPENDS bozo-recomp "${VP2S_ROOT}/tools/vp2_bozo_ui_text_fixture.py" "${VP2S_ROOT}/tools/vp2_bozo_kernel.py" "${VP2S_BOZO_DATA}/images/mod001.elf")
    list(APPEND UI_TEXT_SOURCES "${UI_TEXT_DIR}/ui_text_${VARIANT}_000.cpp" "${UI_TEXT_DIR}/ui_text_${VARIANT}_table.cpp")
    set(UI_GLYPH_DIR "${UI_TEXT_REF}/glyph_${VARIANT}")
    add_custom_command(OUTPUT "${UI_GLYPH_DIR}/ui_glyph_${VARIANT}_000.cpp" "${UI_GLYPH_DIR}/ui_glyph_${VARIANT}_table.cpp" "${UI_GLYPH_DIR}/ui_glyph_${VARIANT}_functions.h"
      COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_ui_text_fixture.py" "${VP2S_BOZO_DATA}/images/mod001.elf" "${UI_GLYPH_DIR}" --build "${VP2S_BOZO_DATA}/.." --variant ${VARIANT} --glyph
      COMMAND $<TARGET_FILE:bozo-recomp> "${UI_GLYPH_DIR}/fixture.elf" "${UI_GLYPH_DIR}" --prefix ui_glyph_${VARIANT} --symbol-prefix ui_glyph_${VARIANT}_
        --files 1 --externals "${UI_GLYPH_DIR}/externals.txt" --no-comments
      DEPENDS bozo-recomp "${VP2S_ROOT}/tools/vp2_bozo_ui_text_fixture.py" "${VP2S_ROOT}/tools/vp2_bozo_kernel.py" "${VP2S_BOZO_DATA}/images/mod001.elf")
    list(APPEND UI_TEXT_SOURCES "${UI_GLYPH_DIR}/ui_glyph_${VARIANT}_000.cpp" "${UI_GLYPH_DIR}/ui_glyph_${VARIANT}_table.cpp")
  endforeach()
  add_executable(vp2s-bozo-ui-text-check "${VP2S_ROOT}/tests/bozo_ui_text_main.cpp" ${UI_TEXT_SOURCES})
  add_dependencies(vp2s-bozo-ui-text-check vp2s-bozo-glyph-fixture)
  target_include_directories(vp2s-bozo-ui-text-check PRIVATE "${UI_TEXT_REF}/original" "${UI_TEXT_REF}/adapted" "${UI_TEXT_REF}/glyph_original" "${UI_TEXT_REF}/glyph_adapted")
  target_link_libraries(vp2s-bozo-ui-text-check PRIVATE vp2s_bozo_kernel bozo_render)
  if(MINGW)
    target_link_options(vp2s-bozo-ui-text-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-ui-text-capture COMMAND vp2s-bozo-ui-text-check "${GLYPH_FIXTURE}")
  set(TRIG_REF "${CMAKE_CURRENT_BINARY_DIR}/trig-reference")
  foreach(VARIANT original adapted)
    set(TRIG_DIR "${TRIG_REF}/${VARIANT}")
    add_custom_command(OUTPUT "${TRIG_DIR}/trig_${VARIANT}_000.cpp" "${TRIG_DIR}/trig_${VARIANT}_table.cpp" "${TRIG_DIR}/trig_${VARIANT}_functions.h"
      COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_trig_fixture.py" "${VP2S_BOZO_DATA}/images/mod001.elf" "${TRIG_DIR}" --build "${VP2S_BOZO_DATA}/.." --variant ${VARIANT}
      COMMAND $<TARGET_FILE:bozo-recomp> "${TRIG_DIR}/fixture.elf" "${TRIG_DIR}" --prefix trig_${VARIANT} --symbol-prefix trig_${VARIANT}_
        --files 1 --externals "${TRIG_DIR}/externals.txt" --no-comments
      DEPENDS bozo-recomp "${VP2S_ROOT}/tools/vp2_bozo_trig_fixture.py" "${VP2S_ROOT}/tools/vp2_bozo_kernel.py" "${VP2S_BOZO_DATA}/images/mod001.elf")
    list(APPEND TRIG_SOURCES "${TRIG_DIR}/trig_${VARIANT}_000.cpp" "${TRIG_DIR}/trig_${VARIANT}_table.cpp")
  endforeach()
  add_executable(vp2s-bozo-trig-check "${VP2S_ROOT}/tests/bozo_trig_main.cpp" ${TRIG_SOURCES})
  target_include_directories(vp2s-bozo-trig-check PRIVATE "${TRIG_REF}/original" "${TRIG_REF}/adapted")
  target_link_libraries(vp2s-bozo-trig-check PRIVATE vp2s_bozo_kernel)
  target_compile_options(vp2s-bozo-trig-check PRIVATE -ffp-contract=off)
  if(MINGW)
    target_link_options(vp2s-bozo-trig-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-sine-cosine COMMAND vp2s-bozo-trig-check)
  set(WORLD_REF "${CMAKE_CURRENT_BINARY_DIR}/world-reference")
  foreach(VARIANT original adapted)
    set(WORLD_DIR "${WORLD_REF}/${VARIANT}")
    add_custom_command(OUTPUT "${WORLD_DIR}/world_${VARIANT}_000.cpp" "${WORLD_DIR}/world_${VARIANT}_table.cpp" "${WORLD_DIR}/world_${VARIANT}_functions.h"
      COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_world_fixture.py" "${VP2S_BOZO_DATA}/images/mod001.elf" "${WORLD_DIR}" --build "${VP2S_BOZO_DATA}/.." --variant ${VARIANT}
      COMMAND $<TARGET_FILE:bozo-recomp> "${WORLD_DIR}/fixture.elf" "${WORLD_DIR}" --prefix world_${VARIANT} --symbol-prefix world_${VARIANT}_
        --files 1 --externals "${WORLD_DIR}/externals.txt" --no-comments
      DEPENDS bozo-recomp "${VP2S_ROOT}/tools/vp2_bozo_world_fixture.py" "${VP2S_ROOT}/tools/vp2_bozo_kernel.py" "${VP2S_BOZO_DATA}/images/mod001.elf")
    list(APPEND WORLD_SOURCES "${WORLD_DIR}/world_${VARIANT}_000.cpp" "${WORLD_DIR}/world_${VARIANT}_table.cpp")
  endforeach()
  add_executable(vp2s-bozo-world-check "${VP2S_ROOT}/tests/bozo_world_main.cpp" ${WORLD_SOURCES})
  target_include_directories(vp2s-bozo-world-check PRIVATE "${WORLD_REF}/original" "${WORLD_REF}/adapted")
  target_link_libraries(vp2s-bozo-world-check PRIVATE vp2s_bozo_kernel)
  target_compile_options(vp2s-bozo-world-check PRIVATE -mavx2 -ffp-contract=off)
  if(MINGW)
    target_link_options(vp2s-bozo-world-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-object-world COMMAND vp2s-bozo-world-check)
  set(PERFORMANCE_REF "${CMAKE_CURRENT_BINARY_DIR}/performance-reference")
  foreach(VARIANT original adapted)
    set(PERFORMANCE_DIR "${PERFORMANCE_REF}/${VARIANT}")
    set(PERFORMANCE_PATCH_ARGS)
    if(VARIANT STREQUAL "adapted")
      set(PERFORMANCE_PATCH_ARGS --patches "${PERFORMANCE_DIR}/patches.txt")
    endif()
    add_custom_command(OUTPUT "${PERFORMANCE_DIR}/performance_${VARIANT}_000.cpp" "${PERFORMANCE_DIR}/performance_${VARIANT}_table.cpp" "${PERFORMANCE_DIR}/performance_${VARIANT}_functions.h"
      COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_performance_fixture.py" "${VP2S_BOZO_DATA}/images/mod001.elf" "${PERFORMANCE_DIR}" --build "${VP2S_BOZO_DATA}/.." --variant ${VARIANT}
      COMMAND $<TARGET_FILE:bozo-recomp> "${PERFORMANCE_DIR}/fixture.elf" "${PERFORMANCE_DIR}" --prefix performance_${VARIANT} --symbol-prefix performance_${VARIANT}_
        --files 1 --externals "${PERFORMANCE_DIR}/externals.txt" ${PERFORMANCE_PATCH_ARGS} --no-comments
      DEPENDS bozo-recomp "${VP2S_ROOT}/tools/vp2_bozo_performance_fixture.py" "${VP2S_ROOT}/tools/vp2_bozo_kernel.py" "${VP2S_BOZO_DATA}/images/mod001.elf")
    list(APPEND PERFORMANCE_SOURCES "${PERFORMANCE_DIR}/performance_${VARIANT}_000.cpp" "${PERFORMANCE_DIR}/performance_${VARIANT}_table.cpp")
  endforeach()
  add_executable(vp2s-bozo-performance-check "${VP2S_ROOT}/tests/bozo_performance_main.cpp" ${PERFORMANCE_SOURCES})
  target_include_directories(vp2s-bozo-performance-check PRIVATE "${PERFORMANCE_REF}/original" "${PERFORMANCE_REF}/adapted")
  target_link_libraries(vp2s-bozo-performance-check PRIVATE bozo_port)
  if(MINGW)
    target_link_options(vp2s-bozo-performance-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-performance-retirement COMMAND vp2s-bozo-performance-check)
  set(FONT_REF "${CMAKE_CURRENT_BINARY_DIR}/font-reference")
  foreach(VARIANT original adapted)
    foreach(KIND init flush defaults)
      set(FONT_DIR "${FONT_REF}/${KIND}/${VARIANT}")
      set(FONT_PATCH_ARGS)
      set(FONT_IMAGE main)
      if(NOT KIND STREQUAL "flush")
        set(FONT_IMAGE mod001)
      endif()
      if(VARIANT STREQUAL "adapted")
        set(FONT_PATCH_ARGS --patches "${FONT_DIR}/patches.txt")
      endif()
      add_custom_command(OUTPUT "${FONT_DIR}/font_${KIND}_${VARIANT}_000.cpp" "${FONT_DIR}/font_${KIND}_${VARIANT}_table.cpp" "${FONT_DIR}/font_${KIND}_${VARIANT}_functions.h"
        COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_font_fixture.py" "${VP2S_BOZO_DATA}/images/${FONT_IMAGE}.elf" "${FONT_DIR}" --build "${VP2S_BOZO_DATA}/.." --kind ${KIND} --variant ${VARIANT}
        COMMAND $<TARGET_FILE:bozo-recomp> "${FONT_DIR}/fixture.elf" "${FONT_DIR}" --prefix font_${KIND}_${VARIANT} --symbol-prefix font_${KIND}_${VARIANT}_
          --files 1 --externals "${FONT_DIR}/externals.txt" ${FONT_PATCH_ARGS} --no-comments
        DEPENDS bozo-recomp "${VP2S_ROOT}/tools/vp2_bozo_font_fixture.py" "${VP2S_ROOT}/tools/vp2_bozo_kernel.py" "${VP2S_BOZO_DATA}/images/${FONT_IMAGE}.elf")
      list(APPEND FONT_SOURCES "${FONT_DIR}/font_${KIND}_${VARIANT}_000.cpp" "${FONT_DIR}/font_${KIND}_${VARIANT}_table.cpp")
      list(APPEND FONT_INCLUDE "${FONT_DIR}")
    endforeach()
  endforeach()
  add_executable(vp2s-bozo-debug-font-check "${VP2S_ROOT}/tests/bozo_font_main.cpp" ${FONT_SOURCES})
  target_include_directories(vp2s-bozo-debug-font-check PRIVATE ${FONT_INCLUDE})
  target_link_libraries(vp2s-bozo-debug-font-check PRIVATE vp2s_bozo_kernel)
  if(MINGW)
    target_link_options(vp2s-bozo-debug-font-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-debug-font-setup COMMAND vp2s-bozo-debug-font-check "${FONT_REF}/init/original/assets")
  set(COMPOSITION_REF "${CMAKE_CURRENT_BINARY_DIR}/composition-reference")
  foreach(VARIANT original adapted)
    set(COMPOSITION_DIR "${COMPOSITION_REF}/${VARIANT}")
    set(COMPOSITION_PATCH_ARGS)
    if(VARIANT STREQUAL "adapted")
      set(COMPOSITION_PATCH_ARGS --patches "${COMPOSITION_DIR}/patches.txt")
    endif()
    add_custom_command(OUTPUT "${COMPOSITION_DIR}/composition_${VARIANT}_000.cpp" "${COMPOSITION_DIR}/composition_${VARIANT}_table.cpp" "${COMPOSITION_DIR}/composition_${VARIANT}_functions.h"
      COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_composition_fixture.py" "${VP2S_BOZO_DATA}/images/main.elf" "${COMPOSITION_DIR}" --build "${VP2S_BOZO_DATA}/.." --variant ${VARIANT}
      COMMAND $<TARGET_FILE:bozo-recomp> "${COMPOSITION_DIR}/fixture.elf" "${COMPOSITION_DIR}" --prefix composition_${VARIANT} --symbol-prefix composition_${VARIANT}_
        --files 1 --externals "${COMPOSITION_DIR}/externals.txt" ${COMPOSITION_PATCH_ARGS} --no-comments
      DEPENDS bozo-recomp "${VP2S_ROOT}/tools/vp2_bozo_composition_fixture.py" "${VP2S_ROOT}/tools/vp2_bozo_kernel.py" "${VP2S_BOZO_DATA}/images/main.elf")
    list(APPEND COMPOSITION_SOURCES "${COMPOSITION_DIR}/composition_${VARIANT}_000.cpp" "${COMPOSITION_DIR}/composition_${VARIANT}_table.cpp")
  endforeach()
  add_executable(vp2s-bozo-composition-check "${VP2S_ROOT}/tests/bozo_composition_main.cpp" ${COMPOSITION_SOURCES})
  target_include_directories(vp2s-bozo-composition-check PRIVATE "${COMPOSITION_REF}/original" "${COMPOSITION_REF}/adapted")
  target_link_libraries(vp2s-bozo-composition-check PRIVATE vp2s_bozo_kernel)
  if(MINGW)
    target_link_options(vp2s-bozo-composition-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-frame-composition COMMAND vp2s-bozo-composition-check)
  set(ASSETS_REF "${CMAKE_CURRENT_BINARY_DIR}/assets-reference")
  foreach(VARIANT original adapted)
    set(ASSETS_DIR "${ASSETS_REF}/${VARIANT}")
    add_custom_command(OUTPUT "${ASSETS_DIR}/assets_${VARIANT}_000.cpp" "${ASSETS_DIR}/assets_${VARIANT}_table.cpp" "${ASSETS_DIR}/assets_${VARIANT}_functions.h"
      COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_assets_fixture.py" "${VP2S_BOZO_DATA}/images/main.elf" "${ASSETS_DIR}" --build "${VP2S_BOZO_DATA}/.." --variant ${VARIANT}
      COMMAND $<TARGET_FILE:bozo-recomp> "${ASSETS_DIR}/fixture.elf" "${ASSETS_DIR}" --prefix assets_${VARIANT} --symbol-prefix assets_${VARIANT}_
        --files 1 --externals "${ASSETS_DIR}/externals.txt" --no-comments
      DEPENDS bozo-recomp "${VP2S_ROOT}/tools/vp2_bozo_assets_fixture.py" "${VP2S_ROOT}/tools/vp2_bozo_kernel.py" "${VP2S_BOZO_DATA}/images/main.elf")
    list(APPEND ASSETS_SOURCES "${ASSETS_DIR}/assets_${VARIANT}_000.cpp" "${ASSETS_DIR}/assets_${VARIANT}_table.cpp")
  endforeach()
  add_executable(vp2s-bozo-assets-cpu-check "${VP2S_ROOT}/tests/bozo_assets_cpu_main.cpp" ${ASSETS_SOURCES})
  target_include_directories(vp2s-bozo-assets-cpu-check PRIVATE "${ASSETS_REF}/original" "${ASSETS_REF}/adapted")
  target_link_libraries(vp2s-bozo-assets-cpu-check PRIVATE vp2s_bozo_kernel)
  if(MINGW)
    target_link_options(vp2s-bozo-assets-cpu-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-asset-producers COMMAND vp2s-bozo-assets-cpu-check)
  foreach(VARIANT original adapted)
    set(TEXTURE_DIR "${TEXTURE_REF}/${VARIANT}")
    set(TEXTURE_PATCH_ARGS --patches "${TEXTURE_DIR}/patches.txt")
    add_custom_command(OUTPUT "${TEXTURE_DIR}/texture_${VARIANT}_000.cpp" "${TEXTURE_DIR}/texture_${VARIANT}_table.cpp" "${TEXTURE_DIR}/texture_${VARIANT}_functions.h"
      COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_texture_fixture.py" "${VP2S_BOZO_DATA}/images/mod001.elf" "${TEXTURE_DIR}" --build "${VP2S_BOZO_DATA}/.." --variant ${VARIANT}
      COMMAND $<TARGET_FILE:bozo-recomp> "${TEXTURE_DIR}/fixture.elf" "${TEXTURE_DIR}" --prefix texture_${VARIANT} --symbol-prefix texture_${VARIANT}_
        --files 1 --externals "${TEXTURE_DIR}/externals.txt" ${TEXTURE_PATCH_ARGS} --no-comments
      DEPENDS bozo-recomp "${VP2S_ROOT}/tools/vp2_bozo_texture_fixture.py" "${VP2S_ROOT}/tools/vp2_bozo_kernel.py" "${VP2S_BOZO_DATA}/images/mod001.elf")
    list(APPEND TEXTURE_SOURCES "${TEXTURE_DIR}/texture_${VARIANT}_000.cpp" "${TEXTURE_DIR}/texture_${VARIANT}_table.cpp")
  endforeach()
  add_executable(vp2s-bozo-textures-check "${VP2S_ROOT}/tests/bozo_textures_main.cpp" ${TEXTURE_SOURCES})
  target_include_directories(vp2s-bozo-textures-check PRIVATE "${TEXTURE_REF}/original" "${TEXTURE_REF}/adapted")
  target_link_libraries(vp2s-bozo-textures-check PRIVATE vp2s_bozo_kernel)
  if(MINGW)
    target_link_options(vp2s-bozo-textures-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-texture-publication COMMAND vp2s-bozo-textures-check "${TEXTURE_REF}/original/assets")
  set(SCREEN_RECT_REF "${CMAKE_CURRENT_BINARY_DIR}/screen-rect-reference")
  foreach(VARIANT original adapted)
    foreach(KIND screen flush completion)
      if(KIND STREQUAL "screen")
        set(SCREEN_IMAGE mod002)
      elseif(KIND STREQUAL "flush")
        set(SCREEN_IMAGE main)
      else()
        set(SCREEN_IMAGE mod001)
      endif()
      set(SCREEN_DIR "${SCREEN_RECT_REF}/${KIND}/${VARIANT}")
      set(SCREEN_PREFIX "rect_${KIND}_${VARIANT}")
      add_custom_command(OUTPUT "${SCREEN_DIR}/${SCREEN_PREFIX}_000.cpp" "${SCREEN_DIR}/${SCREEN_PREFIX}_table.cpp" "${SCREEN_DIR}/${SCREEN_PREFIX}_functions.h"
        COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_screen_rect_fixture.py" "${VP2S_BOZO_DATA}/images/${SCREEN_IMAGE}.elf" "${SCREEN_DIR}" --build "${VP2S_BOZO_DATA}/.." --kind ${KIND} --variant ${VARIANT}
        COMMAND $<TARGET_FILE:bozo-recomp> "${SCREEN_DIR}/fixture.elf" "${SCREEN_DIR}" --prefix ${SCREEN_PREFIX} --symbol-prefix ${SCREEN_PREFIX}_
          --files 1 --externals "${SCREEN_DIR}/externals.txt" --patches "${SCREEN_DIR}/patches.txt" --no-comments
        DEPENDS bozo-recomp "${VP2S_ROOT}/tools/vp2_bozo_screen_rect_fixture.py" "${VP2S_ROOT}/tools/vp2_bozo_kernel.py" "${VP2S_BOZO_DATA}/images/${SCREEN_IMAGE}.elf")
      list(APPEND SCREEN_RECT_SOURCES "${SCREEN_DIR}/${SCREEN_PREFIX}_000.cpp" "${SCREEN_DIR}/${SCREEN_PREFIX}_table.cpp")
      list(APPEND SCREEN_RECT_INCLUDES "${SCREEN_DIR}")
    endforeach()
  endforeach()
  add_executable(vp2s-bozo-screen-rect-check "${VP2S_ROOT}/tests/bozo_screen_rect_main.cpp" ${SCREEN_RECT_SOURCES})
  target_include_directories(vp2s-bozo-screen-rect-check PRIVATE ${SCREEN_RECT_INCLUDES})
  target_link_libraries(vp2s-bozo-screen-rect-check PRIVATE vp2s_bozo_kernel bozo_render)
  if(MINGW)
    target_link_options(vp2s-bozo-screen-rect-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-screen-rectangle COMMAND vp2s-bozo-screen-rect-check)
  set(MATH_REF "${CMAKE_CURRENT_BINARY_DIR}/math-reference")
  foreach(VARIANT original adapted)
    set(MATH_DIR "${MATH_REF}/${VARIANT}")
    add_custom_command(OUTPUT "${MATH_DIR}/math_${VARIANT}_000.cpp" "${MATH_DIR}/math_${VARIANT}_table.cpp" "${MATH_DIR}/math_${VARIANT}_functions.h"
      COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_math_fixture.py" "${VP2S_BOZO_DATA}/images/mod001.elf" "${MATH_DIR}" --build "${VP2S_BOZO_DATA}/.." --variant ${VARIANT}
      COMMAND $<TARGET_FILE:bozo-recomp> "${MATH_DIR}/fixture.elf" "${MATH_DIR}" --prefix math_${VARIANT} --symbol-prefix math_${VARIANT}_
        --files 1 --externals "${MATH_DIR}/externals.txt" --no-comments
      DEPENDS bozo-recomp "${VP2S_ROOT}/tools/vp2_bozo_math_fixture.py" "${VP2S_ROOT}/tools/vp2_bozo_kernel.py" "${VP2S_BOZO_DATA}/images/mod001.elf")
    list(APPEND MATH_SOURCES "${MATH_DIR}/math_${VARIANT}_000.cpp" "${MATH_DIR}/math_${VARIANT}_table.cpp")
  endforeach()
  add_executable(vp2s-bozo-math-check "${VP2S_ROOT}/tests/bozo_math_main.cpp" ${MATH_SOURCES})
  target_include_directories(vp2s-bozo-math-check PRIVATE "${MATH_REF}/original" "${MATH_REF}/adapted")
  target_link_libraries(vp2s-bozo-math-check PRIVATE vp2s_bozo_kernel)
  target_compile_options(vp2s-bozo-math-check PRIVATE -mavx2 -ffp-contract=off)
  if(MINGW)
    target_link_options(vp2s-bozo-math-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-math-records COMMAND vp2s-bozo-math-check)
  set(INSTALL_REF "${CMAKE_CURRENT_BINARY_DIR}/graphics-install-reference")
  foreach(VARIANT original adapted)
    set(INSTALL_DIR "${INSTALL_REF}/${VARIANT}")
    set(INSTALL_PATCH_ARGS --patches "${INSTALL_DIR}/patches.txt")
    add_custom_command(OUTPUT "${INSTALL_DIR}/install_${VARIANT}_000.cpp" "${INSTALL_DIR}/install_${VARIANT}_table.cpp" "${INSTALL_DIR}/install_${VARIANT}_functions.h"
      COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_graphics_fixture.py" "${VP2S_BOZO_DATA}/images/mod001.elf" "${INSTALL_DIR}" --build "${VP2S_BOZO_DATA}/.." --variant ${VARIANT}
      COMMAND $<TARGET_FILE:bozo-recomp> "${INSTALL_DIR}/fixture.elf" "${INSTALL_DIR}" --prefix install_${VARIANT} --symbol-prefix install_${VARIANT}_
        --files 1 --externals "${INSTALL_DIR}/externals.txt" ${INSTALL_PATCH_ARGS} --no-comments
      DEPENDS bozo-recomp "${VP2S_ROOT}/tools/vp2_bozo_graphics_fixture.py" "${VP2S_ROOT}/tools/vp2_bozo_kernel.py" "${VP2S_BOZO_DATA}/images/mod001.elf")
    list(APPEND INSTALL_SOURCES "${INSTALL_DIR}/install_${VARIANT}_000.cpp" "${INSTALL_DIR}/install_${VARIANT}_table.cpp")
  endforeach()
  add_executable(vp2s-bozo-graphics-install-check "${VP2S_ROOT}/tests/bozo_graphics_install_main.cpp" ${INSTALL_SOURCES})
  target_include_directories(vp2s-bozo-graphics-install-check PRIVATE "${INSTALL_REF}/original" "${INSTALL_REF}/adapted")
  target_link_libraries(vp2s-bozo-graphics-install-check PRIVATE vp2s_bozo_kernel)
  if(MINGW)
    target_link_options(vp2s-bozo-graphics-install-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-graphics-install COMMAND vp2s-bozo-graphics-install-check)
  set(FRAME_REF "${CMAKE_CURRENT_BINARY_DIR}/frame-reference")
  foreach(VARIANT original adapted)
    set(FRAME_DIR "${FRAME_REF}/${VARIANT}")
    set(FRAME_PATCH_ARGS)
    if(VARIANT STREQUAL "adapted")
      set(FRAME_PATCH_ARGS --patches "${FRAME_DIR}/patches.txt")
    endif()
    add_custom_command(OUTPUT "${FRAME_DIR}/frame_${VARIANT}_000.cpp" "${FRAME_DIR}/frame_${VARIANT}_table.cpp" "${FRAME_DIR}/frame_${VARIANT}_functions.h"
      COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_frame_fixture.py" "${VP2S_BOZO_DATA}/images/main.elf" "${FRAME_DIR}"
        --build "${VP2S_BOZO_DATA}/.." --variant ${VARIANT}
      COMMAND $<TARGET_FILE:bozo-recomp> "${FRAME_DIR}/fixture.elf" "${FRAME_DIR}" --prefix frame_${VARIANT} --symbol-prefix frame_${VARIANT}_
        --files 1 --externals "${FRAME_DIR}/externals.txt" ${FRAME_PATCH_ARGS} --no-comments
      DEPENDS bozo-recomp "${VP2S_ROOT}/tools/vp2_bozo_frame_fixture.py" "${VP2S_ROOT}/tools/vp2_bozo_kernel.py" "${VP2S_BOZO_DATA}/images/main.elf")
    list(APPEND FRAME_SOURCES "${FRAME_DIR}/frame_${VARIANT}_000.cpp" "${FRAME_DIR}/frame_${VARIANT}_table.cpp")
  endforeach()
  add_executable(vp2s-bozo-frame-check "${VP2S_ROOT}/tests/bozo_frame_main.cpp" ${FRAME_SOURCES})
  target_include_directories(vp2s-bozo-frame-check PRIVATE "${FRAME_REF}/original" "${FRAME_REF}/adapted")
  target_link_libraries(vp2s-bozo-frame-check PRIVATE vp2s_bozo_kernel)
  if(MINGW)
    target_link_options(vp2s-bozo-frame-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-frame-event COMMAND vp2s-bozo-frame-check)
  set_tests_properties(vp2s-bozo-frame-event PROPERTIES TIMEOUT 30)
  set(CSI_REF "${CMAKE_CURRENT_BINARY_DIR}/csi-native-reference")
  add_custom_command(OUTPUT "${CSI_REF}/csi__iop.cpp" "${CSI_REF}/vp2_csi_image.h" "${CSI_REF}/csi.bin"
    COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_sound_fixture.py"
      "${VP2S_BOZO_DATA}/../extract/iop/Csi_.irx" "${CSI_REF}" --kind csi --build "${VP2S_BOZO_DATA}/.."
    DEPENDS "${VP2S_ROOT}/tools/vp2_bozo_sound_fixture.py" "${VP2S_BOZO_DATA}/../extract/iop/Csi_.irx"
      "${VP2S_ROOT}/external/BozoEngine/tools/native_irx/relocate.py" "${VP2S_ROOT}/external/BozoEngine/tools/native_irx/recompile.py")
  add_executable(vp2s-bozo-csi-check "${VP2S_ROOT}/tests/bozo_csi_main.cpp" "${CSI_REF}/csi__iop.cpp")
  target_include_directories(vp2s-bozo-csi-check PRIVATE "${CSI_REF}")
  target_link_libraries(vp2s-bozo-csi-check PRIVATE bozo_libs)
  if(MINGW)
    target_link_options(vp2s-bozo-csi-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-csi-init COMMAND vp2s-bozo-csi-check)
  set_tests_properties(vp2s-bozo-csi-init PROPERTIES TIMEOUT 10)
  add_test(NAME vp2s-bozo-csi-seed-request COMMAND vp2s-bozo-csi-check --request "${CSI_REF}/csi.bin")
  set_tests_properties(vp2s-bozo-csi-seed-request PROPERTIES TIMEOUT 10)
  set(CSD_REF "${CMAKE_CURRENT_BINARY_DIR}/csd-native-reference")
  add_custom_command(OUTPUT "${CSD_REF}/csd__iop.cpp" "${CSD_REF}/vp2_csd_image.h"
    COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_sound_fixture.py"
      "${VP2S_BOZO_DATA}/../extract/iop/Csd_.irx" "${CSD_REF}" --kind csd --build "${VP2S_BOZO_DATA}/.."
    DEPENDS "${VP2S_ROOT}/tools/vp2_bozo_sound_fixture.py" "${VP2S_BOZO_DATA}/../extract/iop/Csd_.irx"
      "${VP2S_ROOT}/external/BozoEngine/tools/native_irx/relocate.py" "${VP2S_ROOT}/external/BozoEngine/tools/native_irx/recompile.py")
  set(CSM_REF "${CMAKE_CURRENT_BINARY_DIR}/csm-native-reference")
  add_custom_command(OUTPUT "${CSM_REF}/csm__iop.cpp" "${CSM_REF}/vp2_csm_image.h" "${CSM_REF}/csm_timer_reference.cpp"
    COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_sound_fixture.py"
      "${VP2S_BOZO_DATA}/../extract/iop/Csm_.irx" "${CSM_REF}" --kind csm --build "${VP2S_BOZO_DATA}/.."
    DEPENDS "${VP2S_ROOT}/tools/vp2_bozo_sound_fixture.py" "${VP2S_BOZO_DATA}/../extract/iop/Csm_.irx"
      "${VP2S_ROOT}/external/BozoEngine/tools/native_irx/relocate.py" "${VP2S_ROOT}/external/BozoEngine/tools/native_irx/recompile.py")
  add_executable(vp2s-bozo-csm-check "${VP2S_ROOT}/tests/bozo_csm_main.cpp" "${CSM_REF}/csm__iop.cpp")
  target_include_directories(vp2s-bozo-csm-check PRIVATE "${CSM_REF}")
  target_link_libraries(vp2s-bozo-csm-check PRIVATE bozo_libs)
  if(MINGW)
    target_link_options(vp2s-bozo-csm-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-csm-init COMMAND vp2s-bozo-csm-check)
  set_tests_properties(vp2s-bozo-csm-init PROPERTIES TIMEOUT 10)
  add_executable(vp2s-bozo-csm-timer-check "${VP2S_ROOT}/tests/bozo_csm_timer_main.cpp"
    "${CSM_REF}/csm__iop.cpp" "${CSM_REF}/csm_timer_reference.cpp")
  target_link_libraries(vp2s-bozo-csm-timer-check PRIVATE bozo_libs)
  if(MINGW)
    target_link_options(vp2s-bozo-csm-timer-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-csm-timer COMMAND vp2s-bozo-csm-timer-check)
  set_tests_properties(vp2s-bozo-csm-timer PROPERTIES TIMEOUT 10)
  add_executable(vp2s-bozo-csd-check "${VP2S_ROOT}/tests/bozo_csd_main.cpp" "${CSM_REF}/csm__iop.cpp" "${CSD_REF}/csd__iop.cpp")
  target_include_directories(vp2s-bozo-csd-check PRIVATE "${CSM_REF}" "${CSD_REF}")
  target_link_libraries(vp2s-bozo-csd-check PRIVATE bozo_libs)
  if(MINGW)
    target_link_options(vp2s-bozo-csd-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-csd-init COMMAND vp2s-bozo-csd-check)
  set_tests_properties(vp2s-bozo-csd-init PROPERTIES TIMEOUT 10)
  set(SOUND_REF "${CMAKE_CURRENT_BINARY_DIR}/sound-native-reference")
  add_custom_command(OUTPUT "${SOUND_REF}/sound_device_library_iop.cpp" "${SOUND_REF}/vp2_sound_image.h"
    COMMAND ${Python3_EXECUTABLE} "${VP2S_ROOT}/tools/vp2_bozo_sound_fixture.py"
      "${VP2S_BOZO_DATA}/../extract/iop/Sound_Device_Library.irx" "${SOUND_REF}" --build "${VP2S_BOZO_DATA}/.."
    DEPENDS "${VP2S_ROOT}/tools/vp2_bozo_sound_fixture.py" "${VP2S_BOZO_DATA}/../extract/iop/Sound_Device_Library.irx"
      "${VP2S_ROOT}/external/BozoEngine/tools/native_irx/relocate.py" "${VP2S_ROOT}/external/BozoEngine/tools/native_irx/recompile.py")
  add_executable(vp2s-bozo-sound-check "${VP2S_ROOT}/tests/bozo_sound_main.cpp" "${SOUND_REF}/sound_device_library_iop.cpp")
  target_include_directories(vp2s-bozo-sound-check PRIVATE "${SOUND_REF}")
  target_link_libraries(vp2s-bozo-sound-check PRIVATE bozo_libs)
  if(MINGW)
    target_link_options(vp2s-bozo-sound-check PRIVATE -static)
  endif()
  add_test(NAME vp2s-bozo-sound-provider COMMAND vp2s-bozo-sound-check)
  set_tests_properties(vp2s-bozo-sound-provider PROPERTIES TIMEOUT 10)
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
  add_executable(vp2s-bozo-headless main.cpp "${DBC_REF}/dbc_manager_iop.cpp" "${CONTROLLER_REF}/ds2o_d_iop.cpp" "${SOUND_REF}/sound_device_library_iop.cpp" "${CSM_REF}/csm__iop.cpp" "${CSD_REF}/csd__iop.cpp" "${CSI_REF}/csi__iop.cpp")
  target_include_directories(vp2s-bozo-headless PRIVATE "${DBC_REF}" "${CONTROLLER_REF}" "${SOUND_REF}" "${CSM_REF}" "${CSD_REF}" "${CSI_REF}")
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
