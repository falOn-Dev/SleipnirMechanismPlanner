import json
import matplotlib.pyplot as plt


def visualize_trajectory(filename):
    # Load trajectory data from JSON file
    with open(filename, 'r') as file:
        data = json.load(file)
    
    # Extract arm and elevator samples
    arm_samples = data['trajectory']['arm_samples']
    elevator_samples = data['trajectory']['elevator_samples']

    # Prepare data for plotting
    arm_times = [sample['t'] for sample in arm_samples]
    arm_positions = [sample['theta'] for sample in arm_samples]
    arm_velocities = [sample['w'] for sample in arm_samples]
    arm_accelerations = [sample['accel'] for sample in arm_samples]

    elevator_times = [sample['t'] for sample in elevator_samples]
    elevator_positions = [sample['height'] for sample in elevator_samples]
    elevator_velocities = [sample['vel'] for sample in elevator_samples]
    elevator_accelerations = [sample['accel'] for sample in elevator_samples]

    # Compute sample counts
    arm_sample_count = len(arm_samples)
    elevator_sample_count = len(elevator_samples)

    # Create plots
    fig, axs = plt.subplots(2, 3, figsize=(12, 8))
    fig.suptitle('Trajectory Visualization')

    # Arm position vs. time
    axs[0, 0].plot(arm_times, arm_positions, label="Arm Position", color="blue")
    axs[0, 0].set_title(f"Arm Position vs Time (Samples: {arm_sample_count})")
    axs[0, 0].set_xlabel("Time (s)")
    axs[0, 0].set_ylabel("Theta (rad)")
    axs[0, 0].legend()
    axs[0, 0].grid(True)

    # Arm velocity vs. time
    axs[0, 1].plot(arm_times, arm_velocities, label="Arm Velocity", color="orange")
    axs[0, 1].set_title(f"Arm Velocity vs Time (Samples: {arm_sample_count})")
    axs[0, 1].set_xlabel("Time (s)")
    axs[0, 1].set_ylabel("Angular Velocity (rad/s)")
    axs[0, 1].legend()
    axs[0, 1].grid(True)

    # Arm acceleration vs. time
    axs[0, 2].plot(arm_times, arm_accelerations, label="Arm Acceleration", color="purple")
    axs[0, 2].set_title(f"Arm Acceleration vs Time (Samples: {arm_sample_count})")
    axs[0, 2].set_xlabel("Time (s)")
    axs[0, 2].set_ylabel("Angular Acceleration (rad/s^2)")
    axs[0, 2].legend()
    axs[0, 2].grid(True)

    # Elevator position vs. time
    axs[1, 0].plot(elevator_times, elevator_positions, label="Elevator Position", color="green")
    axs[1, 0].set_title(f"Elevator Position vs Time (Samples: {elevator_sample_count})")
    axs[1, 0].set_xlabel("Time (s)")
    axs[1, 0].set_ylabel("Height (m)")
    axs[1, 0].legend()
    axs[1, 0].grid(True)

    # Elevator velocity vs. time
    axs[1, 1].plot(elevator_times, elevator_velocities, label="Elevator Velocity", color="red")
    axs[1, 1].set_title(f"Elevator Velocity vs Time (Samples: {elevator_sample_count})")
    axs[1, 1].set_xlabel("Time (s)")
    axs[1, 1].set_ylabel("Velocity (m/s)")
    axs[1, 1].legend()
    axs[1, 1].grid(True)

    # Elevator acceleration vs. time
    axs[1, 2].plot(elevator_times, elevator_accelerations, label="Elevator Acceleration", color="black")
    axs[1, 2].set_title(f"Elevator Acceleration vs Time (Samples: {elevator_sample_count})")
    axs[1, 2].set_xlabel("Time (s)")
    axs[1, 2].set_ylabel("Acceleration (m/s^2)")
    axs[1, 2].legend()
    axs[1, 2].grid(True)

    # Adjust layout and show the plot
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.show()


# Example usage
if __name__ == "__main__":
    # Replace 'trajectory.json' with the path to your JSON file
    visualize_trajectory('build/trajectory.json')
