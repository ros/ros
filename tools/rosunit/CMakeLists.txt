cmake_minimum_required(VERSION 2.8)
project(rosunit)
find_package(catkin REQUIRED)
catkin_package()
catkin_package_export(CFG_EXTRAS ${PROJECT_NAME}-extras.cmake)

install(PROGRAMS
  scripts/test_results_dir.py
  scripts/summarize_results.py
  scripts/rosunit
  scripts/pycoverage_to_html.py
  scripts/clean_junit_xml.py
  scripts/check_test_ran.py
  DESTINATION share/rosunit/scripts)

catkin_add_nosetests(test)
