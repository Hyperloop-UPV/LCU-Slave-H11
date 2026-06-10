cmake_minimum_required(VERSION 3.14)

project(template-project LANGUAGES ASM C CXX)
include(CTest)
enable_testing()

set(EXECUTABLE ${PROJECT_NAME}.elf)
set(STLIB_DIR ${CMAKE_CURRENT_LIST_DIR}/deps/ST-LIB)
set(LD_SCRIPT  ${STLIB_DIR}/STM32H723ZGTX_FLASH.ld)
set(CONTROL_DIR ${CMAKE_CURRENT_LIST_DIR}/deps/LCU-Control-H11)
set(SHARED_DIR  ${CMAKE_CURRENT_LIST_DIR}/deps/LCU-Shared-H11)
set(X_CUBE_AI_DIR ${CMAKE_CURRENT_LIST_DIR}/X-CUBE-AI)
set(X_CUBE_AI_MIDDLEWARE_DIR ${CMAKE_CURRENT_LIST_DIR}/Middlewares/ST/AI)
set(X_CUBE_AI_RUNTIME_LIB
  ${X_CUBE_AI_MIDDLEWARE_DIR}/Lib/GCC/STM32H7/NetworkRuntime1020_CM7_GCC.a
)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

if(CMAKE_HOST_WIN32)
  set(VENV_PYTHON ${CMAKE_SOURCE_DIR}/virtual/Scripts/python)
else()
  set(VENV_PYTHON ${CMAKE_SOURCE_DIR}/virtual/bin/python)
endif()
if(EXISTS "${VENV_PYTHON}")
  set(PYTHON_FOR_TOOLS "${VENV_PYTHON}")
else()
  set(PYTHON_FOR_TOOLS "${Python3_EXECUTABLE}")
endif()

option(USE_ETHERNET "Enable ethernet peripheral" OFF)
option(USE_5_DOF "Build for 5-DOF configuration" OFF)
option(USE_3_DOF "Build for 3-DOF configuration" OFF)
option(USE_DPC_AI "Enable the 1DOF X-CUBE-AI DPC controller" OFF)
option(TARGET_NUCLEO "Targets the STM32H723 Nucleo development board" OFF)
option(USE_CCACHE "Use ccache if available" ON)
if(NOT DEFINED ENABLE_LTO)
  if(CMAKE_CROSSCOMPILING)
    set(ENABLE_LTO OFF)
  else()
    set(ENABLE_LTO ON)
  endif()
endif()
set(ENABLE_LTO "${ENABLE_LTO}" CACHE BOOL "Enable link-time optimization (LTO)")

if(USE_CCACHE)
  find_program(CCACHE_PROGRAM ccache)
  if(CCACHE_PROGRAM)
    set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
    set(CMAKE_ASM_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  endif()
endif()

message(STATUS "Template project: CMAKE_CROSSCOMPILING = ${CMAKE_CROSSCOMPILING}")
message(STATUS "Template project: USE_ETHERNET         = ${USE_ETHERNET}")
message(STATUS "Template project: USE_5_DOF            = ${USE_5_DOF}")
message(STATUS "Template project: USE_3_DOF            = ${USE_3_DOF}")
message(STATUS "Template project: USE_DPC_AI           = ${USE_DPC_AI}")
message(STATUS "Template project: TARGET_NUCLEO        = ${TARGET_NUCLEO}")

set(USE_1_DOF_BUILD OFF)
if(NOT USE_5_DOF AND NOT USE_3_DOF)
  set(USE_1_DOF_BUILD ON)
endif()
if(USE_DPC_AI AND NOT USE_1_DOF_BUILD)
  message(FATAL_ERROR "USE_DPC_AI is only supported for 1DOF builds")
endif()
message(STATUS "Template project: USE_1_DOF_BUILD      = ${USE_1_DOF_BUILD}")

add_subdirectory(${STLIB_DIR})
message(STATUS "Using ST-LIB target: ${STLIB_LIBRARY}")

file(GLOB_RECURSE SOURCE_C    ${CMAKE_SOURCE_DIR}/Core/*.c)
file(GLOB_RECURSE SOURCE_CPP  ${CMAKE_SOURCE_DIR}/Core/*.cpp)
file(GLOB_RECURSE SOURCE_H    ${CMAKE_SOURCE_DIR}/Core/*.h)
file(GLOB_RECURSE SOURCE_HPP  ${CMAKE_SOURCE_DIR}/Core/*.hpp)

# Add Simulink-generated control code
file(GLOB CONTROL_C ${CONTROL_DIR}/*.c ${CONTROL_DIR}/*.cpp ${CONTROL_DIR}/**/*.c ${CONTROL_DIR}/**/*.cpp)
file(GLOB CONTROL_H ${CONTROL_DIR}/*.h ${CONTROL_DIR}/**/*.h)

file(GLOB X_CUBE_AI_C CONFIGURE_DEPENDS ${X_CUBE_AI_DIR}/App/*.c)
file(GLOB X_CUBE_AI_H CONFIGURE_DEPENDS ${X_CUBE_AI_DIR}/App/*.h)


file(GLOB_RECURSE METADATA_SOURCES
  CONFIGURE_DEPENDS
  ${CMAKE_SOURCE_DIR}/Core/*.c
  ${CMAKE_SOURCE_DIR}/Core/*.cpp
  ${CMAKE_SOURCE_DIR}/Core/*.h
  ${CMAKE_SOURCE_DIR}/Core/*.hpp
)

set(METADATA_OUTPUT ${CMAKE_SOURCE_DIR}/Core/Src/Runes/generated_metadata.cpp)
set(METADATA_TEMPLATE ${CMAKE_SOURCE_DIR}/tools/binary_metadata_template.cpp)
set(METADATA_SCRIPT ${CMAKE_SOURCE_DIR}/tools/generate_binary_metadata.py)

list(REMOVE_ITEM METADATA_SOURCES ${METADATA_OUTPUT})

add_custom_command(
  OUTPUT ${METADATA_OUTPUT}
  COMMAND ${PYTHON_FOR_TOOLS} ${METADATA_SCRIPT}
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  DEPENDS
  ${METADATA_SCRIPT}
  ${METADATA_TEMPLATE}
  ${METADATA_SOURCES}
  COMMENT "Generating binary metadata"
)

add_custom_target(generate_binary_metadata
  DEPENDS ${METADATA_OUTPUT}
)

if(CMAKE_CROSSCOMPILING)
  file(GLOB_RECURSE SOURCE_C CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/Core/*.c)
  file(GLOB_RECURSE SOURCE_CPP CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/Core/*.cpp)
  file(GLOB_RECURSE SOURCE_H CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/Core/*.h)
  file(GLOB_RECURSE SOURCE_HPP CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/Core/*.hpp)


  add_executable(${EXECUTABLE}
    ${SOURCE_C}
    ${SOURCE_CPP}
    ${SOURCE_H}
    ${SOURCE_HPP}
    ${CONTROL_C}
    ${CONTROL_H}
    $<$<BOOL:${USE_DPC_AI}>:${X_CUBE_AI_C}>
    $<$<BOOL:${USE_DPC_AI}>:${X_CUBE_AI_H}>

    $<$<BOOL:${CMAKE_CROSSCOMPILING}>:${STLIB_DIR}/startup_stm32h723zgtx.s>
  )

  target_link_libraries(${EXECUTABLE} PRIVATE
    ${STLIB_LIBRARY}
    $<$<BOOL:${USE_DPC_AI}>:${X_CUBE_AI_RUNTIME_LIB}>
  )

  set_target_properties(${EXECUTABLE} PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED YES
    C_STANDARD 17
    C_STANDARD_REQUIRED YES
  )

  target_compile_definitions(${EXECUTABLE} PRIVATE
    $<$<BOOL:${EXAMPLE_SELECTED}>:EXAMPLE_SELECTED>
    $<$<BOOL:${USE_ETHERNET}>:STLIB_ETH>
    $<$<BOOL:${USE_5_DOF}>:USE_5_DOF>
    $<$<BOOL:${USE_3_DOF}>:USE_3_DOF>
    $<$<AND:$<NOT:$<BOOL:${USE_5_DOF}>>,$<NOT:$<BOOL:${USE_3_DOF}>>>:USE_1_DOF>
    $<$<BOOL:${USE_DPC_AI}>:USE_DPC_AI>
    $<IF:$<BOOL:${TARGET_NUCLEO}>,NUCLEO,BOARD>
    $<IF:$<BOOL:${TARGET_NUCLEO}>,HSE_VALUE=8000000,HSE_VALUE=25000000>
  )

  target_compile_options(${EXECUTABLE} PRIVATE
    $<$<BOOL:${CMAKE_CROSSCOMPILING}>:-mcpu=cortex-m7>
    $<$<BOOL:${CMAKE_CROSSCOMPILING}>:-mfpu=fpv5-d16>
    $<$<BOOL:${CMAKE_CROSSCOMPILING}>:-mfloat-abi=hard>
    $<$<BOOL:${CMAKE_CROSSCOMPILING}>:-mthumb>
    $<$<BOOL:${CMAKE_CROSSCOMPILING}>:-specs=nosys.specs>
    $<$<CONFIG:Debug>:-g>
    $<$<CONFIG:RelWithDebInfo>:-g>
    -ffunction-sections
    -fdata-sections
    -fno-exceptions

    -Wno-psabi

    $<$<COMPILE_LANGUAGE:C>:-w>

    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    # $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
    $<$<COMPILE_LANGUAGE:CXX>:-Werror>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-use-cxa-atexit>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
  )

  target_include_directories(${EXECUTABLE} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${SHARED_DIR}/Inc
    ${CONTROL_DIR}
    $<$<BOOL:${USE_DPC_AI}>:${X_CUBE_AI_DIR}/App>
    $<$<BOOL:${USE_DPC_AI}>:${X_CUBE_AI_MIDDLEWARE_DIR}/Inc>
  )

  target_link_options(${EXECUTABLE} PRIVATE
    $<$<BOOL:${CMAKE_CROSSCOMPILING}>:-T${LD_SCRIPT}>
    $<$<BOOL:${CMAKE_CROSSCOMPILING}>:-mcpu=cortex-m7>
    $<$<BOOL:${CMAKE_CROSSCOMPILING}>:-mthumb>
    $<$<BOOL:${CMAKE_CROSSCOMPILING}>:-mfpu=fpv5-d16>
    $<$<BOOL:${CMAKE_CROSSCOMPILING}>:-mfloat-abi=hard>
    $<$<BOOL:${CMAKE_CROSSCOMPILING}>:-specs=nosys.specs>
    $<$<BOOL:${CMAKE_CROSSCOMPILING}>:-lc>
    $<$<BOOL:${CMAKE_CROSSCOMPILING}>:-lm>
    $<$<BOOL:${CMAKE_CROSSCOMPILING}>:-lnosys>
    $<$<BOOL:${CMAKE_CROSSCOMPILING}>:-Wl,-Map=${PROJECT_NAME}.map,--cref>
    $<$<BOOL:${CMAKE_CROSSCOMPILING}>:-Wl,--gc-sections>
  )

  if(ENABLE_LTO)
    set_property(TARGET ${EXECUTABLE} PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)
    set_property(TARGET ${EXECUTABLE} PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO ON)
  endif()

  # Post-build: Copy binary to out/build/latest.elf and create marker for BOARD builds
  add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
    # Create output directory
    COMMAND ${CMAKE_COMMAND} -E make_directory
    ${CMAKE_SOURCE_DIR}/out/build
    # Copy the built binary to out/build/latest.elf
    COMMAND ${CMAKE_COMMAND} -E copy
    $<TARGET_FILE:${EXECUTABLE}>
    ${CMAKE_SOURCE_DIR}/out/build/latest.elf
    COMMENT "Copying ${EXECUTABLE} to out/build/latest.elf"
  )

  # Create or remove marker file for BOARD builds (used by preflash_check.py)
  if(NOT TARGET_NUCLEO)
    add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_SOURCE_DIR}/out/build/board_build_marker
      COMMENT "Creating BOARD build marker for pre-flash check"
    )
  else()
    add_custom_command(TARGET ${EXECUTABLE} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E remove -f ${CMAKE_SOURCE_DIR}/out/build/board_build_marker
      COMMENT "Removing BOARD build marker (NUCLEO build)"
    )
  endif()
endif()

if(PROJECT_IS_TOP_LEVEL)
  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_BINARY_DIR}/compile_commands.json
    ${CMAKE_CURRENT_SOURCE_DIR}/compile_commands.json
  )
endif()
