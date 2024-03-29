task\_executor package
======================

.. contents::


abstract\_step
--------------

.. autoclass:: task_executor.abstract_step.AbstractStep
    :members:
    :undoc-members:
    :show-inheritance:

    .. automethod:: __call__

tasks
-----

.. autoclass:: task_executor.tasks.TaskContext
    :members:

    .. automethod:: __init__

.. autoclass:: task_executor.tasks.Task
    :members:
    :undoc-members:
    :show-inheritance:

actions
-------

.. automodule:: task_executor.actions
    :members:
    :undoc-members:
    :show-inheritance:


beliefs
-------

.. automodule:: task_executor.beliefs
    :members:
    :undoc-members:
    :show-inheritance:

database
--------

.. automodule:: task_executor.database
    :members:
    :undoc-members:
    :show-inheritance:

task_server
-----------

.. automodule:: task_executor.server
    :members:
    :undoc-members:
    :show-inheritance:


Defined actions, ops, tasks, and data
-------------------------------------

.. toctree::
    :maxdepth: 1

    task_executor.tasks
    task_executor.actions
    task_executor.ops
    task_executor.data


Nodes
-------
.. note:: run_action has not been imported to Stretch Robot yet. if you want to run a single action, you will need to create a task for that action for now.
.. toctree::
    :maxdepth: 1

    task_executor.run_task
    .. task_executor.run_action 
    
