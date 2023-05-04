# MPCC
Model Predictive Contouring Controller (MPCC) for Autonomous Racing

Here is a fork of original [**MPCC**](https://github.com/alexliniger/MPCC) from Alex Liniger.

In this fork **MPCC** has **Matlab** and **C++** implementations. **Matlab** implementation is developed for creating one of the control approaches for the Formula Student Driverless car of **Bauman Racing Team**.

**Matlab** version of this rep implements **C++** (not Matlab) version (**fullsize** branch) of Alex Liniger MPCC with some changes:

1. plant (car) model has additional control input (Brakes rate) and another formulation of Pacejka tire model for the lateral slip. 
2. only track borders polytopic constraint and front slip angle constraint are taken into account, tire ellipse constraint will be added later.
3. Cost calculation was adjusted for the new constraints number and new model with new control input.

Later will be implemented soft polytopic constraint on brakes / throttle values, because on the normal car there is no need to accelerate and decelerate at the same time.

## Installation

Clone this rep in your folder:

```bash
git clone https://github.com/Bauman-Racing-Team/MPCC.git
```
### Requirements to run Matlab version of MPCC

1. Ubuntu (Tested on Ubuntu 22.04);
2. Matlab (Tested on Matlab R2022a);
3. [**HPIPM**](https://github.com/giaf/hpipm/tree/master) for solving OCP QP and [**BLASFEO**](https://github.com/giaf/blasfeo) (HPIPM dependency).

### Requirements to run C++ version of MPCC

1. Ubuntu (Tested on Ubuntu 22.04);
2. [**HPIPM**](https://github.com/giaf/hpipm/tree/master) for solving OCP QP and [**BLASFEO**](https://github.com/giaf/blasfeo) (HPIPM dependency);
3. Other dependencies from <C++/README.md>.

### HPIPM and BLASFEO installation with script

1. Open the terminal and navigate inside this rep:
```bash
cd MPCC
```
2. run **install.sh** script to install **HPIPM** and **BLASFEO**:
```bash
./install.sh
```
(Optional steps and required only for those, who want to use **Matlab** version) 

1. Navigate inside hpipm folder and build matlab MEX interface:
```bash
cd external/hpipm/interfaces/matlab_octave/
```
2. open Matlab and run **env.m** script inside Matlab to set variables for HPIPM, then open **compile_mex_all.m** function inside this folder in Matlab and run it too to compile MEX files for Matlab.

## Running (Tested on **Ubuntu 22.04** only!)

All necessary instrictions for running **C++** and **Matlab** versions could be found in their respective directories.