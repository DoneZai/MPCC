# MPCC
Model Predictive Contouring Controller (MPCC) for Autonomous Racing

Here is a fork of original [**MPCC**](https://github.com/alexliniger/MPCC) from Alex Liniger.

In this fork **MPCC** has only **Matlab** implementation and is developed for developing one of the control approaches for the Formula Student driverless car of **Bauman Racing Team**.

**Matlab** version of this rep implements **C++** (not Matlab) version of Alex Liniger MPCC with some changes:

1. plant (car) model has additional control input (Brakes rate) and another formulation of Pacejka tire model for the latteral slip. 
2. only track borders polytopic constraint and front slip angle constraint are taken into account, ellipse constraint will be added later.
3. Cost calculation was adjusted for the new constraints number and new model with new control input.

Later will be implemented soft polytopic constraint on brakes / throttle values, because on the normal car there is no need to accelerate and decelerate at the same time.

## Installation

Clone this rep in your folder:

```bash
git clone https://github.com/Bauman-Racing-Team/MPCC.git
```
### Requirements to run MPCC

1. Ubuntu (Tested on Ubuntu 22.04);
2. Matlab (Tested on Matlab R2022a);
3. [**HPIPM**](https://github.com/giaf/hpipm/tree/master) for solving OCP QP and [**BLASFEO**](https://github.com/giaf/blasfeo) (HPIPM dependency).

### HPIPM and BLASFEO installation with script

1. Open the terminal and navigate inside this rep:
```bash
cd MPCC
```
2. run **install.sh** script to install **HPIPM** and **BLASFEO**:
```bash
./install.sh
```
3. navigate inside hpipm folder and build matlab MEX interface:
```bash
cd external/hpipm/interfaces/matlab_octave/
```
4. open Matlab and run **env.m** script inside Matlab to set variables for HPIPM, then open **compile_mex_all.m** function inside this folder in Matlab and run it too to compile MEX files for Matlab.

## Running (Tested on **Ubuntu 22.04** only!)

1. Open the terminal and navigate inside this rep Matlab folder:
```bash
cd MPCC/Matlab
```
2. source **env.sh** for **HPIPM** solver with command:
```bash
source env.sh
```
3. run Matlab from this terminal;
4. open **simulation.m** script in Matlab and run it. 

## Tracks

By default a track from [**MPCC**](https://github.com/alexliniger/MPCC) **fullsize** branch is used.

To use other tracks uncomment lines in **simulation.m** file **(33,34,35,37,39,40,42)** and comment out lines **(46,47)**.

change track in **trackNameFile** variable.

It could be:
1. **competitions.mat**;
2. **constrictor.mat**;
3. **FSG.mat**;
4. **FSI.mat**;
5. **garden.mat**;
6. **hairpins.mat**:
7. **mess.mat**;
8. **peanut.mat**:
9. **rand.mat**:
10. **rectangle.mat**:
11. **small.mat**:
12. **thin.mat**.

**Be carefull!!!**, on all other tracks than the default track from the [**MPCC**](https://github.com/alexliniger/MPCC) **fullsize** branch **MPCC** performs bad and unstable now. 


 
