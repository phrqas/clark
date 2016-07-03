# The CLARK system

The Constrained pLanning for Autonomy with RisK (CLARK) generates optimal, chance-constrained conditional (temporal) plans.

## Getting started

Download CLARK and its submodules using the following commands

```
git clone git@git.mers.csail.mit.edu:enterprise/clark.git
cd clark
git submodule init
git submodule update
```

Add the CLARK repository to your PYTHONPATH environment variable.

```
export PYTHONPATH=<path-to-clark-root>:$PYTHONPATH
```

If you would like this modification to be permanent, add it to your `.bashrc` file.

### Installing dependencies

Before using CLARK, make sure that the following dependencies are installed.

- [scipy](https://www.scipy.org/) (https://www.scipy.org/)
- [numpy](http://www.numpy.org/) (http://www.numpy.org/)
- [matplotlib](http://matplotlib.org/) (http://matplotlib.org/)
- [pycosat](https://pypi.python.org/pypi/pycosat) (https://pypi.python.org/pypi/pycosat)
- [ipdb](https://pypi.python.org/pypi/ipdb) (https://pypi.python.org/pypi/ipdb)
- [gurobipy](http://www.gurobi.com/) (http://www.gurobi.com/): follow the instructions for requesting an academic license and installing Gurobi.

If you would like to use CLARK's ROS interface (Python and Common Lisp), you should also have the following installed:

- [ROS](http://www.ros.org/) (http://www.ros.org/)
- [roslisp](http://wiki.ros.org/roslisp) (http://wiki.ros.org/roslisp)
- [SBCL](http://www.sbcl.org/) (http://www.sbcl.org/)

If you would like to draw policies as SVG images, make sure to have these installed:

- [GraphViz](http://www.graphviz.org/) (http://www.graphviz.org/)
- [pydot](https://pypi.python.org/pypi/pydot) (https://pypi.python.org/pypi/pydot)

### Components

* RAO*: algorithm responsible for constructing the conditional policies;

* RMPyL: language allowing control programs (and, in the future, also models) to be specified at a high level of abstraction;

* PyTemporal: package containing a number of tools for scheduling under uncertainty.

## Examples

From the `clark` folder

* Help menu
    ```
    ./clark_system.py -h
    ```

* CLARK used as (Durative) PDDL planner

    - This call uses the a duration model function featuring uncontrollable durations

        ```
        ./clark_system.py pddl examples/pddl/domain.pddl examples/pddl/problem.pddl /tmp/clark_pddl_output.tpn --model examples/pddl/boeing_demo_duration_model.py --duration duration_func --schedule --makespan
        ```

        The output should look something like

        ```
        Total elapsed time: 5.759958 s
        Time to optimal value: 2.981791 s
        Iterations till optimal value: 1
        Number of expanded nodes: 10
        Number of evaluated particles: 10
        Optimal value: 10.000000
        Execution risk for optimal value: 0.000160

        Successfully performed STNU reformulation with scheduling risk 100.000000 %!

        This is the schedule:
        	_Event_0x7f3e9007a690: 0.00 s
        	global_start_event: 0.00 s
        	_Event_0x7f3e8f7dcd10: 5.00 s
        	_Event_0x7f3e8f7fc750: 15.16 s
        	_Event_0x7f3e8f797cd0: 20.16 s
        	_Event_0x7f3e8f7c33d0: 25.16 s
        	_Event_0x7f3e8f7d0a90: 30.16 s
        	_Event_0x7f3e8f77c410: 35.16 s
        	_Event_0x7f3e8f7471d0: 47.16 s
        	_Event_0x7f3e8f747e50: 57.16 s
        	_Event_0x7f3e8f6d4590: 69.16 s
        	_Event_0x7f3e8f704450: 79.16 s
        	_Event_0x7f3e8f695190: 79.16 s
        	global_end_event: 79.16 s
        ```

    - This call uses the a duration model function with only controllable durations.

        ```
        ./clark_system.py pddl examples/pddl/domain.pddl examples/pddl/problem.pddl /tmp/clark_pddl_output.tpn --model examples/pddl/boeing_demo_duration_model.py --duration duration_func_controllable --schedule --makespan
        ```

        The output should look something like

        ```
        Total elapsed time: 5.533587 s
        Time to optimal value: 2.826540 s
        Iterations till optimal value: 1
        Number of expanded nodes: 10
        Number of evaluated particles: 10
        Optimal value: 10.000000
        Execution risk for optimal value: 0.000160

        WARNING: policy has been manually altered.

        Successfully performed STNU reformulation with scheduling risk 0.000000 %!

        This is the schedule:
        	_Event_0x7ff64b096850: 0.00 s
        	global_start_event: 0.00 s
        	_Event_0x7ff64b096910: 2.00 s
        	_Event_0x7ff64a7e5910: 2.00 s
        	_Event_0x7ff64b081ed0: 2.00 s
        	_Event_0x7ff64b081f90: 2.00 s
        	_Event_0x7ff64a802e90: 4.00 s
        	_Event_0x7ff64a7e59d0: 4.00 s
        	_Event_0x7ff64a802f50: 6.00 s
        	_Event_0x7ff64a7af590: 6.00 s
        	_Event_0x7ff64a7bb850: 8.00 s
        	_Event_0x7ff64a7af650: 8.00 s
        	_Event_0x7ff64a7bb450: 10.00 s
        	_Event_0x7ff64a731450: 10.00 s
        	_Event_0x7ff64a73f590: 10.00 s
        	_Event_0x7ff64a73fb10: 10.00 s
        	_Event_0x7ff64a6d15d0: 10.00 s
        	_Event_0x7ff64a731390: 10.00 s
        	_Event_0x7ff64a766990: 10.00 s
        	_Event_0x7ff64a7664d0: 10.00 s
        	global_end_event: 24.00 s
        	_Event_0x7ff64a6edc50: 24.00 s
        	_Event_0x7ff64a6ed950: 24.00 s
        	_Event_0x7ff64a6d1150: 24.00 s
        ```

        - This call performs a chance-constrained STNU reformulation (risk bound of 1%) with makespan optimization on the output PTPN and makes it pseudo-controllable (uncontrollable durations are deemed controllable).

            ```
            ./clark_system.py pddl examples/pddl/domain.pddl examples/pddl/problem.pddl /tmp/clark_pddl_output.tpn --model examples/pddl/boeing_demo_duration_model.py --duration duration_func --schedule --stnuref 1 1 0.01
            ```

            The output should look something like

            ```
            Total elapsed time: 5.539674 s
            Time to optimal value: 2.821624 s
            Iterations till optimal value: 1
            Number of expanded nodes: 10
            Number of evaluated particles: 10
            Optimal value: 10.000000
            Execution risk for optimal value: 0.000160

            WARNING: policy has been manually altered.

            Successfully performed STNU reformulation with scheduling risk 0.000000 %!

            This is the schedule:
            	global_start_event: 0.00 s
            	_Event_0x7f3a56aba850: 0.00 s
            	_Event_0x7f3a56aba910: 2.00 s
            	_Event_0x7f3a56a86ed0: 2.00 s
            	_Event_0x7f3a561ea910: 2.00 s
            	_Event_0x7f3a56a86f90: 2.00 s
            	_Event_0x7f3a561ea9d0: 4.00 s
            	_Event_0x7f3a56207e90: 4.00 s
            	_Event_0x7f3a561b4590: 6.00 s
            	_Event_0x7f3a56207f50: 6.00 s
            	_Event_0x7f3a561b4650: 8.00 s
            	_Event_0x7f3a561c0850: 8.00 s
            	_Event_0x7f3a5616b4d0: 10.00 s
            	_Event_0x7f3a5616b990: 10.00 s
            	_Event_0x7f3a560d6390: 10.00 s
            	_Event_0x7f3a56136450: 10.00 s
            	_Event_0x7f3a561c0450: 10.00 s
            	_Event_0x7f3a56144dd0: 10.00 s
            	_Event_0x7f3a56136390: 10.00 s
            	_Event_0x7f3a56144c50: 10.00 s
            	global_end_event: 24.00 s
            	_Event_0x7f3a560d6850: 24.00 s
            	_Event_0x7f3a56101bd0: 24.00 s
            	_Event_0x7f3a56101710: 24.00 s
            ```

* CLARK used with cRMPL (RMPyl) input

    - This example makes optimal choices without scheduling

        ```
        ./clark_system.py rmpyl examples/rmpyl/baxter_collaborative.py baxter_rmpyl_example_1 /tmp/clark_rmpyl_output.tpn
        ```

        The output should look something like

        ```
        Generating RMPyL program.
        Done in 5.092 s

        ##### Plan stats:
        Name: run()
        Events: 4164
        Primitive episodes: 1583
        Choices: 499
        Temporal constraints: 6178

        ##### Starting RAO* search!

        Iter: 1, Open nodes: 1, States evaluted: 1, Root value: 60.3000, Root ER: 0.0000
        Iter: 2, Open nodes: 5, States evaluted: 1, Root value: 24.2100, Root ER: 0.0000
        Iter: 3, Open nodes: 17, States evaluted: 5, Root value: 7.9560, Root ER: 0.0000
        Iter: 4, Open nodes: 41, States evaluted: 17, Root value: 6.9237, Root ER: 0.0000
        Iter: 5, Open nodes: 49, States evaluted: 41, Root value: 6.2370, Root ER: 0.0000
        Iter: 6, Open nodes: 25, States evaluted: 49, Root value: 5.6070, Root ER: 0.0000
        Iter: 7, Open nodes: 8, States evaluted: 25, Root value: 4.8510, Root ER: 0.0000
        Iter: 8, Open nodes: 4, States evaluted: 8, Root value: 4.2210, Root ER: 0.0000
        Iter: 9, Open nodes: 10, States evaluted: 4, Root value: 3.9123, Root ER: 0.0000
        Iter: 10, Open nodes: 12, States evaluted: 10, Root value: 3.7800, Root ER: 0.0000
        Iter: 11, Open nodes: 6, States evaluted: 12, Root value: 3.7800, Root ER: 0.0000
        Iter: 12, Open nodes: 1, States evaluted: 6, Root value: 3.7800, Root ER: 0.0000
        Iter: 13, Open nodes: 0, States evaluted: 1, Root value: 3.7800, Root ER: 0.0000

        Total elapsed time: 0.822195 s
        Time to optimal value: 0.753379 s
        Iterations till optimal value: 10
        Number of expanded nodes: 180
        Number of evaluated particles: 180
        Optimal value: 3.780000
        Execution risk for optimal value: 0.000000
        ```

    - This example makes optimal choices and performs risk-aware conditional scheduling.

        ```
        ./clark_system.py rmpyl examples/rmpyl/baxter_collaborative.py baxter_rmpyl_example_2 /tmp/clark_rmpyl_output.tpn --schedule
        ```

        The output should look something like

        ```
        Generating RMPyL program.
        Done in 0.035 s

        ##### Plan stats:
        Name: run()
        Events: 88
        Primitive episodes: 33
        Choices: 11
        Temporal constraints: 130

        ##### Starting RAO* search!

        Iter: 1, Open nodes: 1, States evaluted: 1, Root value: 0.9000, Root ER: 0.0000
        Iter: 2, Open nodes: 3, States evaluted: 1, Root value: 0.6300, Root ER: 0.0000
        Iter: 3, Open nodes: 3, States evaluted: 3, Root value: 0.6300, Root ER: 0.0000
        Iter: 4, Open nodes: 1, States evaluted: 3, Root value: 0.6300, Root ER: 0.0833
        Iter: 5, Open nodes: 0, States evaluted: 1, Root value: 0.6300, Root ER: 0.2778

        Total elapsed time: 0.242119 s
        Time to optimal value: 0.036945 s
        Iterations till optimal value: 2
        Number of expanded nodes: 9
        Number of evaluated particles: 9
        Optimal value: 0.630000
        Execution risk for optimal value: 0.277830
        ```


## CLARK ROS

The `clark_ros` folder provides a ROS interface for CLARK. In order to be able to use it, we suggest creating a symbolic link to `clark_ros` from inside your existing catkin workspace.

```
ln -s <path-to-clark-ros>/clark_ros <path-to-catkin-ws>/src/clark_ros
```

Also, do not forget to rebuild the workspace in order to generate the CLARK messages! Depending on whether you use the new `catkin_tools` or not, you will be able to build your workspace with `catkin_make` or `catkin build`.

### Python server

Start the CLARK server by issuing.

```
rosrun clark_ros clark_server.py
```

This creates "clark_service", a ROS service that accepts requests with the following fields:

* (string) **type**: service request type, which can be either
    - "pddl";
    - "rmpyl" or "crmpl";
    - "ccpomdp".

* (string) **params**: parameters in command line format. For further information about these, please refer to the help message generated by `clark_system -h`.

### Python client

The `clark_client.py` module defines the `CLARKClient` class, which can be used to interface with the CLARK server described before. Here is an example on how to use this class, which can be also found in `clark_client.py`.

```
import rospy
rospy.init_node('clark_client_node')
rospy.loginfo('Initializing CLARK client')
clark_client = CLARKClient()

clark_path = clark.__path__[0] #Path to CLARK module

stype = 'pddl'
params = '%s %s %s --model %s --duration %s --schedule --makespan --svg'%(
             join(clark_path,'examples/pddl/domain.pddl'),
             join(clark_path,'examples/pddl/problem.pddl'),
             '/tmp/clark_pddl_output.tpn',
             join(clark_path,'examples/pddl/boeing_demo_duration_model.py'),
             'duration_func')

clark_client.request_service(stype,params)

rospy.loginfo('Press Ctrl+C to terminate.')
rospy.spin()
rospy.loginfo('CLARK client terminated')
```

### Common Lisp client

Before you use the Common Lisp client, make sure you have up-to-date versions of [SBCL](http://www.sbcl.org/) and [roslisp](http://wiki.ros.org/roslisp) (the versions that ship by default with Ubuntu might not be sufficient).

First, load `roslisp` and `clark-ros` into your Lisp environment, replacing `<ros-distro>` by your ROS distribution:

```
(load "/opt/ros/<ros-distro>/share/roslisp/scripts/roslisp-sbcl-init") ;;Initializes roslisp
(ros-load:load-system :clark-ros)
```

Next, create the client object:

```
(in-package :roslisp)
(defvar clark-cli)
(setf clark-cli (make-instance 'clark-cl-client))
```

Use the client object to call the CLARK service. Here, we assume that the content of the folder `examples/pddl/` was copied to `/tmp`:

```
(defvar params)
(setf params "/tmp/domain.pddl /tmp/problem.pddl /tmp/clark_pddl_output.tpn --model /tmp/boeing_demo_duration_model.py --duration duration_func --schedule --makespan --svg")
(call-clark clark-cli :type "pddl" :params params)
```

The server should return a message containing a success flag and a string containing the TPN.
