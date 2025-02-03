import argparse
import os
from generate_parameter_library_py.parse_yaml import GenerateCode
import sys


def run(output_file, yaml_file, validation_module=""):
    gen_param_struct = GenerateCode("python")
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    gen_param_struct.parse(yaml_file, validation_module)
    with open(output_file, "w") as f:
        f.write(str(gen_param_struct))


def find_package_name():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    while script_dir != "/":
        if "package.xml" in os.listdir(script_dir):
            return os.path.basename(script_dir)
        script_dir = os.path.dirname(script_dir)
    print("Package not found. Make sure your workspace is sourced correctly.")
    sys.exit(1)


def find_workspace_name():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    while script_dir != "/":
        if all(
            os.path.isdir(os.path.join(script_dir, d))
            for d in ["src", "install", "build", "log"]
        ):
            return os.path.basename(script_dir)
        script_dir = os.path.dirname(script_dir)
    print("Workspace not found. Make sure your workspace is structured correctly.")
    sys.exit(1)


def find_parameters_dir():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    while script_dir != "/":
        if all(os.path.isdir(os.path.join(script_dir, d)) for d in ["params"]):
            return os.path.basename(script_dir)
        script_dir = os.path.dirname(script_dir)
    print(
        "Parameters directory not found. Make sure your workspace is structured correctly."
    )
    sys.exit(1)


def get_workspace_directory(current_path):

    path_parts = current_path.split(os.sep)

    if "install" in path_parts:
        workspace_index = path_parts.index("install") - 1
    elif "src" in path_parts:
        workspace_index = path_parts.index("src") - 1
    else:
        raise ValueError("Workspace directory not found in the path")

    workspace_directory = os.sep.join(path_parts[: workspace_index + 1])
    return workspace_directory


if __name__ == "__main__":

    pkg_name = find_package_name()

    workspace_directory = get_workspace_directory(os.path.abspath(__file__))

    workspace_name = find_workspace_name()

    parameters_dir = find_parameters_dir()

    install_dir = os.path.join(
        "install",
        pkg_name,
        "lib",
        pkg_name,
        "sine_wave_parameters.py",
    )
    params_dir = os.path.join(workspace_directory, "src", "sine_wave_pkg", "params")

    run(
        os.path.join(workspace_directory, install_dir),
        os.path.join(
            os.path.join(params_dir, "sine_wave_prams_sturcture.yaml"),
        ),
    )
