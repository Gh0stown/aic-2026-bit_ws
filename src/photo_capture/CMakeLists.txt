cmake_minimum_required(VERSION 3.0.2)
project(photo_capture)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/capture_photos.py
  scripts/count_video.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  scripts/yolo_detect.py
  scripts/scan.py
  scripts/stitch.py
  scripts/chassis.py
  scripts/gauge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
