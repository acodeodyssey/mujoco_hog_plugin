find_package(rostest REQUIRED)

add_rostest_gtest(mujoco_hog_test
  launch/mujoco_hog_test.test
  mujoco_hog_test.cpp
)

add_dependencies(mujoco_hog_test
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(mujoco_hog_test
  ${catkin_LIBRARIES}
  mujoco_hog
)
