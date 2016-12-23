
set (SRCS main.cpp)

add_executable(${PROJECT_NAME}_app ${SRCS})

target_link_libraries(${PROJECT_NAME}_app
  ${PROJECT_NAME}_lib
  )

install(TARGETS ${PROJECT_NAME}_app DESTINATION ${LIBRARY_INSTALL_PATH})
