add_executable(aimer_module_test module_test.cpp)
target_link_libraries(aimer_module_test PRIVATE xr)
target_compile_features(aimer_module_test PRIVATE cxx_std_20)
add_test(NAME aimer_module_test COMMAND aimer_module_test)

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

add_executable(aimer_bullet_speed_test bullet_speed_test.cpp)
target_include_directories(aimer_bullet_speed_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)
target_compile_features(aimer_bullet_speed_test PRIVATE cxx_std_20)
add_test(NAME aimer_bullet_speed_test COMMAND aimer_bullet_speed_test)

add_executable(aimer_lead_calibration_test lead_calibration_test.cpp)
target_include_directories(aimer_lead_calibration_test PRIVATE ${CMAKE_CURRENT_LIST_DIR}/..)
target_compile_features(aimer_lead_calibration_test PRIVATE cxx_std_20)
add_test(NAME aimer_lead_calibration_test COMMAND aimer_lead_calibration_test)
