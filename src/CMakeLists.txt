file (GLOB_RECURSE SRCS *.cpp)

add_library(${PROJECT_NAME}_lib SHARED ${SRCS})

target_link_libraries(${PROJECT_NAME}_lib
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${OpenNI2_LIBRARIES}
  ${PCL_LIBRARIES}
  )

install(TARGETS ${PROJECT_NAME}_lib DESTINATION ${LIBRARY_INSTALL_PATH})
