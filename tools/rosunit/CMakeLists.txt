cmake_minimum_required(VERSION 3.0.2)
project(rosunit)
find_package(catkin REQUIRED)
catkin_package(CFG_EXTRAS ${PROJECT_NAME}-extras.cmake)

catkin_python_setup()

catkin_install_python(PROGRAMS
  scripts/check_test_ran.py
  scripts/clean_junit_xml.py
  scripts/pycoverage_to_html.py
  scripts/summarize_results.py
  scripts/test_results_dir.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
