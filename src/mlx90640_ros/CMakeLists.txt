cmake_minimum_required(VERSION 3.0.2)
project(mlx90640_ros C)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs)
catkin_python_setup()
catkin_package(LIBRARIES mlx90640 CATKIN_DEPENDS rospy sensor_msgs)

add_library(mlx90640 SHARED vendor/MLX90640_API.c vendor/mlx_wrap.c)
target_include_directories(mlx90640 PRIVATE vendor)
target_compile_options(mlx90640 PRIVATE -O2 -Wall)
target_link_libraries(mlx90640 m)

catkin_install_python(PROGRAMS
  scripts/mlx90640_node.py
  scripts/mlx90640_viewer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS mlx90640
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY test/data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
install(FILES README.md THIRD_PARTY.md LICENSE LICENSE-APACHE-2.0
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING AND NOSETESTS)
  set(test_result_dir ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME})
  catkin_run_tests_target(nosetests core nosetests-core.xml
    # catkin's test wrapper expects each shell command as one string.
    COMMAND "\"${CMAKE_COMMAND}\" -E make_directory \"${test_result_dir}\""
      "\"${CMAKE_COMMAND}\" -E env \"MLX90640_LIBRARY=$<TARGET_FILE:mlx90640>\" \"PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/src:$ENV{PYTHONPATH}\" \"${NOSETESTS}\" -v \"${CMAKE_CURRENT_SOURCE_DIR}/test\" --with-xunit \"--xunit-file=${test_result_dir}/nosetests-core.xml\""
    DEPENDENCIES mlx90640
  )
endif()
