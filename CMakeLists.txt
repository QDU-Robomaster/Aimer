# Aimer 运行时模块

target_include_directories(xr PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/tinympc
)

set(MODULE_AIMER_SRC
  "${CMAKE_CURRENT_LIST_DIR}/Aimer.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/tinympc/admm.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/tinympc/rho_benchmark.cpp"
  "${CMAKE_CURRENT_LIST_DIR}/tinympc/tiny_api.cpp"
)

target_sources(xr PRIVATE ${MODULE_AIMER_SRC})

get_filename_component(_MODULE_DIRNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
set(_DEPS_TARGET "${_MODULE_DIRNAME}_deps")
add_library(${_DEPS_TARGET} INTERFACE)

set_property(GLOBAL APPEND PROPERTY XR_MODULE_DEPS ${_DEPS_TARGET})
