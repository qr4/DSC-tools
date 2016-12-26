file (GLOB_RECURSE SRCS src/*.cpp)
include_directories(include)

set (APP_NAME ht_vr)

add_executable(${PROJECT_NAME}_${APP_NAME} ${SRCS})

target_link_libraries(${PROJECT_NAME}_${APP_NAME}
  ${PROJECT_NAME}_lib
  )

install(TARGETS ${PROJECT_NAME}_${APP_NAME} DESTINATION ${LIBRARY_INSTALL_PATH})
