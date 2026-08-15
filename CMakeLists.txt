# Aimer 运行时模块

target_include_directories(xr PUBLIC ${CMAKE_CURRENT_LIST_DIR}
                                     ${CMAKE_CURRENT_LIST_DIR}/tinympc)

set(MODULE_AIMER_SRC
    "${CMAKE_CURRENT_LIST_DIR}/tinympc/admm.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/tinympc/rho_benchmark.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/tinympc/tiny_api.cpp")

target_sources(xr PRIVATE ${MODULE_AIMER_SRC})

get_filename_component(_MODULE_DIRNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
set(_DEPS_TARGET "${_MODULE_DIRNAME}_deps")
add_library(${_DEPS_TARGET} INTERFACE)

find_package(OpenCV 4 REQUIRED COMPONENTS core calib3d)

target_include_directories(${_DEPS_TARGET} INTERFACE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${_DEPS_TARGET} INTERFACE ${OpenCV_LIBS})

set_property(GLOBAL APPEND PROPERTY XR_MODULE_DEPS ${_DEPS_TARGET})

if(BUILD_TESTING)
  include(CTest)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/tests
                   ${CMAKE_CURRENT_BINARY_DIR}/Aimer_tests)
endif()
