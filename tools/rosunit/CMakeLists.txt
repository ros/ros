cmake_minimum_required(VERSION 2.8)
project(rosunit)
find_package(catkin REQUIRED)
catkin_package(CFG_EXTRAS ${PROJECT_NAME}-extras.cmake)

catkin_python_setup()

install(PROGRAMS
  scripts/check_test_ran.py
  scripts/clean_junit_xml.py
  scripts/pycoverage_to_html.py
  scripts/summarize_results.py
  scripts/test_results_dir.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

catkin_add_nosetests(test)
