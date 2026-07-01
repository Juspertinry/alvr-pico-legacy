cmake_minimum_required(VERSION 3.22.1)
project(p2alvr)

# This is a perf-critical VR client. The Gradle `debug` variant otherwise compiles
# the native lib at -O0 (CMAKE_*_FLAGS_DEBUG default) which leaves the whole ~6k-line
# compositor unoptimized. Force -O2 -DNDEBUG regardless of variant. -DNDEBUG also
# compiles out the verbose #ifndef NDEBUG dev logging in jni.cpp.
set(CMAKE_C_FLAGS_DEBUG   "-O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-O2 -DNDEBUG")

# Import the prebuilt Pico SDK native library so we can link against its
# exported C symbols (Pvr_Init, Pvr_GetMainSensorState, GetRenderEventFunc, ...).
add_library(Pvr_UnitySDK SHARED IMPORTED)
set_target_properties(Pvr_UnitySDK PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../jniLibs/${ANDROID_ABI}/libPvr_UnitySDK.so)

# Import the prebuilt ALVR client_core (Rust) C API library.
add_library(alvr_client_core SHARED IMPORTED)
set_target_properties(alvr_client_core PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/../jniLibs/${ANDROID_ABI}/libalvr_client_core.so)

add_library(p2alvr SHARED
    render_thread.cpp
    gl_util.cpp
    eye_tracking.cpp
    device_info.cpp
    ui_kit.cpp
    eq_panel.cpp
    app_state.cpp
    lobby_panels.cpp
    menu_model.cpp
    settings_panel.cpp
    input.cpp
    foveation.cpp
    jni.cpp)

find_library(log-lib log)

# Thin LTO on OUR translation units + link only (imported prebuilt libs are
# unaffected -- they're IMPORTED, never compiled here).
target_compile_options(p2alvr PRIVATE -flto=thin)
target_link_options(p2alvr PRIVATE -flto=thin)

target_link_libraries(p2alvr
    Pvr_UnitySDK
    alvr_client_core
    GLESv3
    EGL
    android
    ${log-lib})
