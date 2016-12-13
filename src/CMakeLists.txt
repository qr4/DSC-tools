
file (GLOB_RECURSE SRCS *.cpp)

add_library(${PROJECT_NAME} SHARED ${SRCS})

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  )

install(TARGETS ${PROJECT_NAME} DESTINATION ${LIBRARY_INSTALL_PATH})
