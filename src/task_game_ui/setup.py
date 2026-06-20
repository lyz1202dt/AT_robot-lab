from setuptools import setup

package_name = "task_game_ui"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/task_game_ui.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lyz",
    maintainer_email="lyz@todo.todo",
    description="Tkinter UI for publishing task_game box_id_grid values.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task_game_ui_node = task_game_ui.task_game_ui_node:main",
        ],
    },
)
