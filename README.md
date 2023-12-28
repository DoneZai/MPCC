# MPCC
Model Predictive Contouring Controller (MPCC) for Autonomous Racing

Here is a fork of original [**MPCC**](https://github.com/alexliniger/MPCC) from Alex Liniger.

In this fork **MPCC** has **Matlab** and **C++** implementations. **Matlab** implementation is developed for creating one of the control approaches for the Formula Student Driverless car of **Bauman Racing Team**.

In the current **Matlab** version new cost function, constraints functions were suggested. Moreover **Matlab** version does not use [**hpipm**](https://github.com/giaf/hpipm) solver anymore, [**acados**](https://github.com/acados/acados) and [**ipopt**](https://coin-or.github.io/Ipopt/) solvers with [**CasADi**](https://web.casadi.org/) interface for automatic equations differentiation are used instead.

Now a primary version uses **acados**, because it has lots of benefits for solving OCPs in a real time applications. **ipopt** version is used as a reference and on the current version have some problems, which will be solved soon.

![racing lap on the FSG track](https://github.com/Bauman-Racing-Team/MPCC/blob/develop/Matlab/race_FSG_track.gif)
![racing lap on the FSI track](https://github.com/Bauman-Racing-Team/MPCC/blob/develop/Matlab/race_FSI_track.gif)
![racing lap on the thin track](https://github.com/Bauman-Racing-Team/MPCC/blob/develop/Matlab/race_thin_track.gif)

## Installation

Clone this rep in your folder:

```bash
git clone https://github.com/Bauman-Racing-Team/MPCC.git
```
### Requirements to run Matlab version of MPCC

1. Ubuntu (Tested on Ubuntu 22.04);
2. Matlab (Tested on Matlab R2022a);
3. [**CasADi**](https://web.casadi.org/) for automatic differentiation and **ipopt** solver, which comes with **CasADi**;
4. [**acados**](https://github.com/acados/acados) for solving OCP in a real time.

### Requirements to run C++ version of MPCC

1. Ubuntu (Tested on Ubuntu 22.04);
2. [**HPIPM**](https://github.com/giaf/hpipm/tree/master) for solving OCP QP and [**BLASFEO**](https://github.com/giaf/blasfeo) (HPIPM dependency);
3. Other dependencies from <C++/README.md>.

### Requirements installation with a script, which will install all requirenments for both versions Matlab and C++

1. Open the terminal and navigate inside this rep:
```bash
cd MPCC
```
2. run **install.sh** script to install **acados** and **CasADi** with **ipopt** for the **Matlab** version and **HPIPM** and **BLASFEO** for the **C++** version:
```bash
./install.sh
```

## Running (Tested on **Ubuntu 22.04** only!)

All necessary instrictions for running **C++** and **Matlab** versions could be found in their respective directories.
