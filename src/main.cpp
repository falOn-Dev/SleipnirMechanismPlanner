#include "Eigen/Core"
#include "sleipnir/autodiff/Variable.hpp"
#include "sleipnir/optimization/SolverExitCondition.hpp"
#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <numbers>
#include <numeric>
#include <stdexcept>
#include <sleipnir/optimization/OptimizationProblem.hpp>

using namespace std::chrono_literals;

void saveTrajectoryToJson(const Eigen::MatrixXd &armSamples,
                          const Eigen::MatrixXd &elevatorSamples,
                          const Eigen::MatrixXd &armAccel,
                          const Eigen::MatrixXd &elevatorAccel,
                          nlohmann::json &jsonTrajectory, double dt) {
  nlohmann::json armSamplesArray = nlohmann::json::array();
  for (int col = 0; col < armSamples.cols(); ++col) {
    armSamplesArray.push_back({{"t", col * dt},
                               {"theta", armSamples(0, col)},
                               {"w", armSamples(1, col)},
                               {"accel", armAccel(0, col)}});
  }

  nlohmann::json elevatorSamplesArray = nlohmann::json::array();
  for (int col = 0; col < elevatorSamples.cols(); ++col) {
    elevatorSamplesArray.push_back({{"t", col * dt},
                                    {"height", elevatorSamples(0, col)},
                                    {"vel", elevatorSamples(1, col)},
                                    {"accel", elevatorAccel(0, col)}});
  }

  jsonTrajectory["trajectory"] = {
      {"dt", dt},
      {"arm_samples", armSamplesArray},
      {"elevator_samples", elevatorSamplesArray},
  };
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    throw std::invalid_argument("Usage: ./executable trajectory.json");
  }

  std::string filename = argv[1];

  // Read input JSON
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::ios_base::failure("Failed to open file: " + filename);
  }
  nlohmann::json jsonTrajectory;
  file >> jsonTrajectory;
  file.close();

  // Extract constants
  double elevatorMaxHeight = jsonTrajectory["constants"]["elevator_length"];
  double armLength = jsonTrajectory["constants"]["arm_length"];

  // Extract start and end conditions
  double elevatorStartHeight = jsonTrajectory["elevator"]["start_height"];
  double elevatorEndHeight = jsonTrajectory["elevator"]["end_height"];
  double armStartAngle = static_cast<double>(jsonTrajectory["arm"]["start_angle"]) * std::numbers::pi / 180.0;
  double armEndAngle = static_cast<double>(jsonTrajectory["arm"]["end_angle"]) * std::numbers::pi / 180.0;

  constexpr int N = 800;
  constexpr double elevatorMaxVelocity = 1.0;     // m/s
  constexpr double elevatorMaxAcceleration = 2.0; // m/s^2
  constexpr double armMaxVelocity = 2.0 * std::numbers::pi;     // rad/s
  constexpr double armMaxAcceleration = 4.0 * std::numbers::pi; // rad/s^2

  sleipnir::OptimizationProblem problem;

  auto elevator = problem.DecisionVariable(2, N + 1);
  auto elevatorAccel = problem.DecisionVariable(1, N);

  auto arm = problem.DecisionVariable(2, N + 1);
  auto armAccel = problem.DecisionVariable(1, N);

  auto dt = problem.DecisionVariable(N);

  for (int k = 0; k < N; k++) {
    problem.SubjectTo(elevator(0, k + 1) ==
                      elevator(0, k) + elevator(1, k) * dt(k) +
                          0.5 * elevatorAccel(0, k) * dt(k) * dt(k));

    problem.SubjectTo(elevator(1, k + 1) ==
                      elevator(1, k) + elevatorAccel(0, k) * dt(k));

    problem.SubjectTo(arm(0, k + 1) ==
                      arm(0, k) + arm(1, k) * dt(k) +
                          0.5 * armAccel(0, k) * dt(k) * dt(k));

    problem.SubjectTo(arm(1, k + 1) == arm(1, k) + armAccel(0, k) * dt(k));
  }

  problem.SubjectTo(dt >= 0.0);
  problem.SubjectTo(dt <= 0.2);

  for (int k = 0; k < N - 1; k++) {
    problem.SubjectTo(dt(k) == dt(k + 1));
    dt(k).SetValue(0.05);
  }

  problem.SubjectTo(elevator.Col(0) ==
                    Eigen::Vector2d({elevatorStartHeight, 0.0}));
  problem.SubjectTo(elevator.Col(N) ==
                    Eigen::Vector2d({elevatorEndHeight, 0.0}));

  problem.SubjectTo(arm.Col(0) == Eigen::Vector2d({armStartAngle, 0.0}));
  problem.SubjectTo(arm.Col(N) == Eigen::Vector2d({armEndAngle, 0.0}));

  problem.SubjectTo(-elevatorMaxVelocity <= elevator.Row(1));
  problem.SubjectTo(elevator.Row(1) <= elevatorMaxVelocity);

  problem.SubjectTo(-elevatorMaxAcceleration <= elevatorAccel);
  problem.SubjectTo(elevatorAccel <= elevatorMaxAcceleration);

  problem.SubjectTo(elevator.Row(0) <= elevatorMaxHeight);
  problem.SubjectTo(elevator.Row(0) >= 0.0);

  problem.SubjectTo(-armMaxVelocity <= arm.Row(1));
  problem.SubjectTo(arm.Row(1) <= armMaxVelocity);

  problem.SubjectTo(-armMaxAcceleration <= armAccel);
  problem.SubjectTo(armAccel <= armMaxAcceleration);

  problem.Minimize(
      std::accumulate(dt.begin(), dt.end(), sleipnir::Variable{0.0}) +
      1e-5 * (elevatorAccel * elevatorAccel.T())(0) +
      1e-5 * (armAccel * armAccel.T())(0));

  auto solution = problem.Solve({.diagnostics = true});

  std::println("dt = {}", dt.Value(0));

  auto elevatorSamples = elevator.Value();
  auto armSamples = arm.Value();

  saveTrajectoryToJson(armSamples, elevatorSamples, armAccel.Value(),
                       elevatorAccel.Value(), jsonTrajectory, dt.Value(0));

  // Save updated JSON
  std::ofstream outputFile(filename);
  if (!outputFile.is_open()) {
    throw std::ios_base::failure("Failed to open file for writing: " + filename);
  }
  outputFile << jsonTrajectory.dump(4);
  outputFile.close();
}
