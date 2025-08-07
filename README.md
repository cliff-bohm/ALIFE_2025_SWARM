# ALIFE_2025_SWARM
This repository contains the files needed to replicate the work shown in "Disentangling Swarm Behavior and Function: Improved Explainability via Information-Flow Diagrams and Controller Activation Patterns" as presented at ALIFE 2025

This includes the tasks ("Worlds"), maxNavWorld(Nav World in the paper) and slimBerryWorld (berry world in the paper).

# MABE Installation Guide

## Step 1: Ensure Required Components (you do not need to install MABE yet)

Visit the [Installation Guide](https://github.com/Hintzelab/MABE/wiki/Installation-and-getting-started-with-MABE) to check required software:
- C++17 Compiler
- CMake >= 3.13.3
- Python >= 3.7

## Step 2: Build MABE

1. Download this repository.
2. Navigate to the top-level directory.
3. Run `sh tools/setup.cmd` to set up mbuild.
4. Run `./mbuild` (or `./mbuild.exe` for Windows).
5. The executable will be placed in the `work/` directory.

## Step 3: test MABE build

1. Navigate to the `work/` directory:
    ```sh
    cd work/
    ```
2. Execute MABE using:
    ```sh
    ./mabe
    ```
on windows, use: `./mabe.exe`

Two directories are included in this repository, nav_world and berry_world, which have been configured with the parameters settings used with each of these tasks. To run in one of these enviernemtns, navigate to either directory.


1. Copy MABE from the work directory:
    ```sh
    cp ../work/mabe* .
    ```

2. Execute MABE using:
    ```sh
    python ../tools/mq.py -l
    ```

For further details on how `mq.py` works, refer to the [MQ documentation](https://github.com/Hintzelab/MABE/wiki/MQ).

Information regaurding parameters can be found in `work/settings_world.cfg`
To run different configurations changes can be made to `work/mq_conditions.txt`



## Useful Links

- [MABE GitHub Repository](https://github.com/Hintzelab/MABE/)
- [Installation and Getting Started Guide](https://github.com/Hintzelab/MABE/wiki/Installation-and-getting-started-with-MABE)
- [MQ Documentation](https://github.com/Hintzelab/MABE/wiki/MQ)

