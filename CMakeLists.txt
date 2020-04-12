cmake_minimum_required(VERSION 3.1...3.17)

if(${CMAKE_VERSION} VERSION_LESS 3.12)
  cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()

project(
  omplgp
  VERSION 0.1.0
  DESCRIPTION "An OMPL-based global planner."
  LANGUAGES CXX
)

find_package (ompl)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${OMPL_INCLUDE_DIRS})
target_link_libraries (${PROJECT_NAME} ${OMPL_LIBRARIES} Eigen3::Eigen)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES ${PROJECT_NAME}Config.cmake DESTINATION lib/cmake/${PROJECT_NAME})
install(
  FILES
  include/${PROJECT_NAME}/${PROJECT_NAME}.hpp
  DESTINATION include/${PROJECT_NAME}
)

option(BUILD_TESTS "Build the tests" ON)
if(BUILD_TESTS)
  enable_testing()
  include(GoogleTest)
  add_subdirectory(tests)
endif()

SET(CPACK_GENERATOR "DEB")
SET(
  CPACK_DEBIAN_PACKAGE_MAINTAINER "Abdul Rahman Dabbour <ardabbour@gmail.com>"
)
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libompl-dev (>= 1.2.1+ds1-1build1)")
INCLUDE(CPack)