
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['task_executor'],
    scripts=['scripts/example_task_client.py', 'scripts/fifo_task_executor.py', 'scripts/scheduled_task_executor.py', 'scripts/test_task_action.py'
    , 'scripts/continuous_patrolling.py', 'scripts/task_routine_node.py', 'scripts/example_task_routine.py'],
    package_dir={'': 'src'}
)

setup(**d)