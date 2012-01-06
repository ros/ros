cmake_minimum_required(VERSION 2.8)
project(roscreate)

install(FILES manifest.xml
  DESTINATION share/roscreate
  )
install(DIRECTORY templates/
  DESTINATION share/roscreate
  )
