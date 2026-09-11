cmake_minimum_required(VERSION 3.10)
project(nanompx VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -O2)

option(NANOMPX_WITH_SRT "Build optional SRT transport" ON)
option(NANOMPX_BUILD_TOOLS "Build CLI tools" ON)
option(NANOMPX_BUILD_TESTS "Build unit tests" ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(NANOMPX_SOURCES
    src/frame.c
    src/codec_pcm.c
    src/codec_compressed.c
    src/encoder.c
    src/decoder.c
    src/clock.c
)

add_library(nanompx ${NANOMPX_SOURCES})
target_link_libraries(nanompx PUBLIC m)
target_include_directories(nanompx PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Optional SRT
set(NANOMPX_HAVE_SRT 0)
if(NANOMPX_WITH_SRT)
    find_path(SRT_INCLUDE_DIR NAMES srt/srt.h
        PATHS
            /usr/include
            /usr/local/include
            $ENV{NANOMPX_SRT_ROOT}/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../kunix/libsrt/include
        NO_DEFAULT_PATH
    )
    find_path(SRT_INCLUDE_DIR NAMES srt/srt.h)

    # Prefer shared system libsrt (static kunix archive needs C++/OpenSSL glue).
    find_library(SRT_LIBRARY NAMES srt
        PATHS
            /usr/lib/x86_64-linux-gnu
            /usr/lib
            /usr/local/lib
            $ENV{NANOMPX_SRT_ROOT}/lib
        NO_DEFAULT_PATH
    )
    if(NOT SRT_LIBRARY AND EXISTS "/usr/lib/x86_64-linux-gnu/libsrt.so.1")
        set(SRT_LIBRARY "/usr/lib/x86_64-linux-gnu/libsrt.so.1")
    endif()
    find_library(SRT_LIBRARY NAMES srt)

    if(SRT_INCLUDE_DIR AND SRT_LIBRARY)
        set(NANOMPX_HAVE_SRT 1)
        enable_language(CXX)
        target_sources(nanompx PRIVATE src/srt_transport.c)
        target_include_directories(nanompx PRIVATE ${SRT_INCLUDE_DIR})
        target_link_libraries(nanompx PUBLIC ${SRT_LIBRARY} pthread)
        # Shared libsrt may still need C++ runtime depending on packaging.
        target_link_libraries(nanompx PUBLIC stdc++)
        target_compile_definitions(nanompx PUBLIC NANOMPX_WITH_SRT=1)
        message(STATUS "NanoMPX: SRT enabled (${SRT_LIBRARY})")
    else()
        message(STATUS "NanoMPX: SRT not found; building without SRT transport")
        target_compile_definitions(nanompx PUBLIC NANOMPX_WITH_SRT=0)
    endif()
else()
    target_compile_definitions(nanompx PUBLIC NANOMPX_WITH_SRT=0)
endif()

if(NANOMPX_BUILD_TOOLS)
    add_executable(nanompx_cli tools/nanompx_cli.c)
    target_link_libraries(nanompx_cli PRIVATE nanompx)
endif()

if(NANOMPX_BUILD_TESTS)
    enable_testing()
    add_executable(test_roundtrip tests/unit/test_roundtrip.c)
    target_link_libraries(test_roundtrip PRIVATE nanompx)
    add_test(NAME roundtrip COMMAND test_roundtrip)

    add_executable(test_clock_sfn tests/unit/test_clock_sfn.c)
    target_link_libraries(test_clock_sfn PRIVATE nanompx)
    add_test(NAME clock_sfn COMMAND test_clock_sfn)

    add_executable(test_compressed tests/unit/test_compressed.c)
    target_link_libraries(test_compressed PRIVATE nanompx)
    add_test(NAME compressed COMMAND test_compressed)
endif()

install(TARGETS nanompx
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
if(NANOMPX_BUILD_TOOLS)
    install(TARGETS nanompx_cli RUNTIME DESTINATION bin)
endif()

# pkg-config
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/nanompx.pc.in
               ${CMAKE_CURRENT_BINARY_DIR}/nanompx.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/nanompx.pc DESTINATION lib/pkgconfig)
