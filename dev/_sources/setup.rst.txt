Setup Guide
===========

Rtd-code-matlab expects MATLAB R2021b or later due to its use of classes and the arguments block in function definitions.

General Setup
-------------

1. In MATLAB R2021b, add `CORA 2021 <https://tumcps.github.io/CORA/pages/archive/v2021/index.html>`_  and its subfolders to your MATLAB path.
2. Add your URDFs folder (and subfolders) to your MATLAB path.
3. Add the rtd-code-matlab folder and subfolders to your MATLAB path.
4. Navigate to the rtd-code-matlab folder and run the script in ``scripts/armour/new_agent_demo.m`` to see if everything is working.

Optional Components for MEXed Functions
---------------------------------------

We also provide MEXed versions of the robust controller used in ARMOUR.
To use these MEXed functions, you will need to install the following build dependencies:

* Eigen3 >= 3.3
* Boost (particularly the `Boost Interval Arthmetic library <https://www.boost.org/doc/libs/1_84_0/libs/numeric/interval/doc/interval.htm>`_)

Obtaining URDFs
---------------

To obtain the URDFs used in the demos, please visit the `ARMOUR <https://github.com/roahmlab/armour>`_ repository and download the ``urdfs`` folder.
These can be added to your path to use the demos in rtd-code-matlab.
