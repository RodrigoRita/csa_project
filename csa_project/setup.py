import os
from glob import glob

from setuptools import find_packages, setup

package_name = "csa_project"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Install launch and extra files
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
        (os.path.join("share", package_name, "world"), glob("world/*.yaml")),
        (os.path.join("share", package_name, "model"), glob("model/*.yaml")),
        (os.path.join("share", package_name, "rl_model"), glob("rl_model/*.zip")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=[
        "setuptools",
        "stable-baselines3",
        "gymnasium",
        "numpy",
    ],
    zip_safe=True,
    maintainer="vscode",
    maintainer_email="vscode@todo.todo",
    description="CSA project using Flatland",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main_node = csa_project.main:main",
            "factory_gui = csa_project.factory_gui:main",
            "OrderAgent = csa_project.OrderAgent:main",
            "ProductAgent = csa_project.ProductAgent:main",
            "ResourceAgent = csa_project.ResourceAgent:main",
            "TransportAgent = csa_project.TransportAgent:main",
            "ResourceController = csa_project.ResourceController:main",
            "execute_skill_action = csa_project.execute_skill_action:main",
            "set_product_color = csa_project.set_product_color_service:main",
            "rl_action = csa_project.rl_action_server:main",
        ],
    },
)
