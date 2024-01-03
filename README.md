# `eagerx_franka` package

[![license](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![codestyle](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)
[![Continuous Integration](https://github.com/eager-dev/eagerx_franka/actions/workflows/ci.yml/badge.svg?branch=master)](https://github.com/eager-dev/eagerx_franka/actions/workflows/ci.yml)
[![Test Coverage](coverage.svg)](https://github.com/eager-dev/eagerx_franka/actions/workflows/ci.yml)


What is the `eagerx_franka` package
---------------------------------

This repository interfaces franka robots with EAGERx. 
EAGERx (Engine Agnostic Graph Environments for Robotics) enables users to easily define new tasks, switch from one sensor to another, and switch from simulation to reality with a single line of code by being invariant to the physics engine.

[The core repository is available here.](https://github.com/eager-dev/eagerx)

[Full documentation and tutorials (including package creation and contributing) are available here.](https://eagerx.readthedocs.io/en/master/)

Installation
------------

You can install the package using pip:

```bash
pip3 install eagerx_franka
```

Requirements for usage with real Franka robot:
- [ros noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [franka_human_friendly_controllers](https://github.com/franzesegiovanni/franka_human_friendly_controllers) see [installation instructions](https://github.com/tud-phi/franka-emika-guide)


Credits
-------

The code within `eagerx_franka.panda_ros` is copied from [panda_ros_py](https://github.com/platonics-delft/panda-ros-py) and from [quaternion_algebra](https://github.com/franzesegiovanni/quaternion_algebra).


Cite EAGERx
-----------

If you are using EAGERx for your scientific publications, please cite:

``` {.sourceCode .bibtex}
@article{eagerx,
    author  = {van der Heijden, Bas and Luijkx, Jelle, and Ferranti, Laura and Kober, Jens and Babuska, Robert},
    title = {EAGERx: Engine Agnostic Graph Environments for Robotics},
    year = {2022},
    publisher = {GitHub},
    journal = {GitHub repository},
    howpublished = {\url{https://github.com/eager-dev/eagerx}}
}
```

Acknowledgements
----------------

EAGERx is funded by the [OpenDR](https://opendr.eu/) Horizon 2020
project.