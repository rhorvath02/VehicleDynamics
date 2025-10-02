import os
import sys
import subprocess

def get_git_root():
    try:
        root = subprocess.check_output(
            ["git", "rev-parse", "--show-toplevel"], text=True
        ).strip()
        return root
    except subprocess.CalledProcessError:
        raise Exception("You don't seem to have git installed. How did you clone this repo??")

def set_model_name_in_mos(file_path: str, model_name: str):
    with open(file_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    with open(file_path, 'w', encoding='utf-8') as f:
        for line in lines:
            if line.strip().startswith('modelName :='):
                f.write(f'modelName := "{model_name}";\n')
            else:
                f.write(line)

script_path = "./VehicleDynamics/Utilities/FMI/generate_FMU.mos"

# Validation
if len(sys.argv) < 2:
    exec_name = os.path.splitext(os.path.basename(sys.executable))[0]
    print(f"Usage: {exec_name} generate_FMU.py <model_path (modelica format)> <destination file path>")
    sys.exit(1)

# Store dest now that it exists
model_path = sys.argv[1]
dest = sys.argv[2]

set_model_name_in_mos(file_path=script_path, model_name=model_path)

# Run the .mos script with omc
try:
    result = subprocess.run(["omc", "./VehicleDynamics/Utilities/FMI/generate_FMU.mos"], capture_output=True, text=True, check=True)

    print("=== STDOUT ===")
    print(result.stdout)

    print("=== STDERR ===")
    print(result.stderr)

except subprocess.CalledProcessError as e:
    print("OMC returned an error:")
    print(e.stdout)
    print(e.stderr)
    sys.exit(1)

except FileNotFoundError:
    print("Error: 'omc' not found. Is OpenModelica installed and in your PATH?")
    sys.exit(1)

# set_model_name_in_mos(file_path=script_path, model_name="")