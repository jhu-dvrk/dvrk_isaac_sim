from pathlib import Path

from setuptools import find_packages, setup


package_name = "dvrk_isaac_sim"
script_files = [
    "scripts/simulator.py",
    "scripts/convert_dvrk_model.py",
    "scripts/generate_cart_frames.py",
    "scripts/validate_config.py",
    "scripts/benchmark_interactive.py",
]

data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
    (f"share/{package_name}/share", ["share/isaac_sim.yaml.example"]),
    (f"share/{package_name}/share/open-xr", [
        str(path) for path in sorted(Path("share/open-xr").glob("*"))
        if path.is_file()
    ]),
    (f"share/{package_name}/share/dvrk_systems", [
        str(path) for path in sorted(Path("share/dvrk_systems").glob("*.json"))
    ]),
    (f"share/{package_name}/launch", [
        "launch/simulator.launch.py",
        "launch/open_xr.launch.py",
        "launch/test_scene.launch.py",
    ]),
    (f"share/{package_name}/scripts", script_files),
]

local_config = Path("share/isaac_sim.yaml")
if local_config.is_file():
    data_files.append((f"share/{package_name}/share", [str(local_config)]))

scene_files = [str(path) for path in sorted(Path("share/scenes").glob("*.yaml"))]
if scene_files:
    data_files.append((f"share/{package_name}/share/scenes", scene_files))


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    maintainer="Anton Deguet",
    maintainer_email="anton.deguet@jhu.edu",
    description="ROS 2 and Isaac Sim kinematic simulation for dVRK PSMs and ECMs.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "simulator_node = dvrk_isaac_sim.node:main",
            "clean_cache = dvrk_isaac_sim.clean_cache:main",
        ],
    },
)
