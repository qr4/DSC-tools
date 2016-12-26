
set (SRCS main.cpp)

set (APP_NAME example_app)

add_executable(${PROJECT_NAME}_${APP_NAME} ${SRCS})

target_link_libraries(${PROJECT_NAME}_${APP_NAME}
  ${PROJECT_NAME}_lib
  )

install(TARGETS ${PROJECT_NAME}_${APP_NAME} DESTINATION ${LIBRARY_INSTALL_PATH})
