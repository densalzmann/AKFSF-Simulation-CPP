cmake_minimum_required(VERSION 3.5)
project(AKFSFSimulation)

set(CMAKE_CXX_STANDARD 11)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(SRC_CPP
    src/main.cpp
    src/display.cpp
    src/beacons.cpp
    src/sensors.cpp
    src/simulation.cpp
    src/utils.cpp
)

function(filter_conf type)
add_executable("${PROJECT_NAME}_${type}"
        ${SRC_CPP}
        "src/kalmanfilter_${type}.cpp"
        )
target_include_directories( "${PROJECT_NAME}_${type}"
        PUBLIC ./src)
target_link_libraries("${PROJECT_NAME}_${type}"
        SDL2
        SDL2_ttf
        Eigen3::Eigen
        )
endfunction(filter_conf)

filter_conf(lkf)
filter_conf(ekf)
filter_conf(ukf)

add_executable(${PROJECT_NAME}
        ${SRC_CPP}
        src/kalmanfilter.cpp
        )


target_include_directories( "${PROJECT_NAME}"
        PUBLIC ./src)

target_link_libraries("${PROJECT_NAME}"
        SDL2
        SDL2_ttf
        Eigen3::Eigen
        )

add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/data/ $<TARGET_FILE_DIR:${PROJECT_NAME}>)
    
    