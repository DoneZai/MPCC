## Running instructions for Matlab version of MPCC

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

To use splined **FSG** track from [**nirajbasnet**](https://github.com/nirajbasnet/Nonlinear_MPCC_for_autonomous_racing) rep uncomment lines in **somulation.m** file **(51,52,53,55,56,58,59,61,62,64)** and comment out lines **(33,34,35,37,39,40,42)** and **(46,47)**

**Be carefull!!!**, on all other tracks than the default track from the [**MPCC**](https://github.com/alexliniger/MPCC) **fullsize** branch **MPCC** performs bad and unstable for now. 