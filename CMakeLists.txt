cmake_minimum_required(VERSION 3.22)

# Set the project name
project(SleipnirElevatorArmPlanner)

include(FetchContent)

fetchcontent_declare(
    Sleipnir
    GIT_REPOSITORY https://github.com/SleipnirGroup/Sleipnir
    GIT_TAG main
)
fetchcontent_makeavailable(Sleipnir)

# Define the nlohmann/json library
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3 # Specify the version you want to use
)
FetchContent_MakeAvailable(nlohmann_json)


# Add an executable
add_executable(SleipnirElevatorArmPlanner src/main.cpp)

target_link_libraries(SleipnirElevatorArmPlanner PUBLIC Sleipnir nlohmann_json)
