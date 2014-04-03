
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    # packages=['task_executor'],
    scripts=['scripts/example_task_client.py', 'scripts/task_executor_node.py', 'scripts/test_task_action.py'],
    # package_dir={'': 'src'}
)

setup(**d)