add_executable(aimer_preview_geometry_test aimer_preview_geometry_test.cpp)

target_link_libraries(aimer_preview_geometry_test PRIVATE xr)
target_compile_features(aimer_preview_geometry_test PRIVATE cxx_std_20)

add_test(NAME aimer_preview_geometry_test COMMAND aimer_preview_geometry_test)

add_executable(aimer_stage_frame_access_test stage_frame_access_test.cpp)
target_link_libraries(aimer_stage_frame_access_test PRIVATE xr)
target_compile_features(aimer_stage_frame_access_test PRIVATE cxx_std_20)

add_test(NAME aimer_stage_frame_access_test COMMAND aimer_stage_frame_access_test)

add_executable(aimer_referee_topic_test referee_topic_test.cpp)
target_link_libraries(aimer_referee_topic_test PRIVATE xr)
target_compile_features(aimer_referee_topic_test PRIVATE cxx_std_20)
add_test(NAME aimer_referee_default_topic_test COMMAND aimer_referee_topic_test
                                                       robot_game_ref
)
add_test(NAME aimer_referee_sentry_topic_test COMMAND aimer_referee_topic_test sentry_ref)

add_executable(aimer_heat_fire_test heat_fire_test.cpp)
target_include_directories(aimer_heat_fire_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)
target_compile_features(aimer_heat_fire_test PRIVATE cxx_std_20)
add_test(NAME aimer_heat_fire_test COMMAND aimer_heat_fire_test)
