import csv
import matplotlib.pyplot as plt
import os


def load_csv(path):
    t, vcmd, cpos, x, v, done = [], [], [], [], [], []
    with open(path, "r") as f:
        r = csv.DictReader(f)
        for row in r:
            t.append(float(row["t_s"]))
            vcmd.append(float(row["v_cmd"]))
            cpos.append(float(row["control_pos"]))
            x.append(float(row["actual_pos"]))
            v.append(float(row["actual_vel"]))
            done.append(int(row["done"]))
    return t, vcmd, cpos, x, v, done


# Define the path to the build directory with CSV files
build_dir = "../../../build/test/planner_tests"

# Define all the CSV files and their descriptive titles
csv_files = [
    ("output_integrator_nonzero_vf.csv", "Integrator with Non-Zero Final Velocity"),
    ("output_integrator_velocity_mode.csv", "Integrator in Velocity Mode"),
    ("output_preempt_integrator_early.csv", "Preempted Integrator (Early)"),
    ("output_preempt_integrator_late.csv", "Preempted Integrator (Late)"),
    ("output_preempt_integrator_mid.csv", "Preempted Integrator (Mid)"),
    ("output_preempt_secondorder_early.csv", "Preempted Second-Order (Early)"),
    ("output_preempt_secondorder_late.csv", "Preempted Second-Order (Late)"),
    ("output_preempt_secondorder_mid.csv", "Preempted Second-Order (Mid)"),
    ("output_second_order_integrator.csv", "Second-Order Integrator Plant"),
    ("output_second_order_position.csv", "Second-Order Position Plant"),
]

# Create corpus directory path
corpus_dir = "../corpus"
os.makedirs(corpus_dir, exist_ok=True)

# Create plots for each CSV file
for fname, title in csv_files:
    csv_path = os.path.join(build_dir, fname)

    # Check if file exists
    if not os.path.exists(csv_path):
        print(f"Warning: File {csv_path} not found, skipping...")
        continue

    try:
        t, vcmd, cpos, x, v, done = load_csv(csv_path)

        # Create safe filename for saving
        safe_name = fname.replace(".csv", "").replace("output_", "")

        # Create position plot
        plt.figure(figsize=(10, 6))
        plt.title(f"{title} — Position")
        plt.scatter(t, x, label="actual_pos", marker=".", alpha=0.7)
        plt.scatter(t, cpos, label="control_pos", marker=".", alpha=0.7)
        plt.xlabel("time [s]")
        plt.ylabel("position")
        plt.legend()
        plt.grid(True)

        # Save position plot
        pos_filename = os.path.join(corpus_dir, f"{safe_name}_position.png")
        plt.savefig(pos_filename, dpi=150, bbox_inches="tight")

        # Create velocity plot
        plt.figure(figsize=(10, 6))
        plt.title(f"{title} — Velocity")
        plt.scatter(t, v, label="actual_vel", marker=".", alpha=0.7)
        plt.scatter(t, vcmd, label="velocity_command", marker=".", alpha=0.7)
        plt.xlabel("time [s]")
        plt.ylabel("velocity")
        plt.legend()
        plt.grid(True)

        # Save velocity plot
        vel_filename = os.path.join(corpus_dir, f"{safe_name}_velocity.png")
        plt.savefig(vel_filename, dpi=150, bbox_inches="tight")

        print(f"Generated plots for {fname}: {safe_name}_position.png, {safe_name}_velocity.png")

    except Exception as e:
        print(f"Error processing {fname}: {e}")

print("All plots generated successfully!")
print(f"Plots saved to {corpus_dir}/")
plt.close("all")  # Close all figures to free memory
