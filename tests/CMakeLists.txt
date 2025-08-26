add_executable(smoke_test
    smoke_test/smoke_test.cpp
    ${CMAKE_SOURCE_DIR}/src/obstacle_processor.cpp   
    ${CMAKE_SOURCE_DIR}/src/global_planning.cpp
    ${CMAKE_SOURCE_DIR}/src/local_planning.cpp
    ${CMAKE_SOURCE_DIR}/src/trajectory_planning.cpp
    ${CMAKE_SOURCE_DIR}/src/utils.cpp
)



target_include_directories(smoke_test PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(smoke_test PRIVATE Eigen3::Eigen KDTreeLib)
target_compile_features(smoke_test PRIVATE cxx_std_17)

add_test(NAME smoke_test COMMAND smoke_test)
set_tests_properties(smoke_test PROPERTIES
  ENVIRONMENT "OBSTACLE_DATA_FILE=${CMAKE_SOURCE_DIR}/data/obstacle-data.csv"
)