cmake_minimum_required(VERSION 3.0.2)
project(photo_capture)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/capture_photos.py
  scripts/count_video.py
  scripts/inspection_vision.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  scripts/yolo_detect.py
  scripts/scan.py
  scripts/stitch.py
  scripts/chassis.py
  scripts/gauge.py
  scripts/gauge_exp16.py
  scripts/gauge_exp16_stability.py
  scripts/thermal_calibration.py
  scripts/building_windows.py
  scripts/ocr_engine.py
  scripts/iv_common.py
  scripts/iv_service.py
  scripts/iv_tasks_detect.py
  scripts/iv_tasks_trash.py
  scripts/iv_tasks_ebike.py
  scripts/iv_tasks_ocr.py
  scripts/iv_tasks_thermal.py
  scripts/iv_tasks_meter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY scripts/gauge_exp16_vendor/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/gauge_exp16_vendor
  FILES_MATCHING PATTERN "*.py" PATTERN "__pycache__" EXCLUDE
)

install(FILES models/gauge_exp16.pt
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  OPTIONAL
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)
