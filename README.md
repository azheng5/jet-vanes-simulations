This is the full reposiory for the simulations for the Jet Vanes rocket for GT Guidance, Navigation, and Controls. 

## Prerequisites

The following software tools are required for contributing to the jet vanes simulations repository:
- [MATLAB](https://www.mathworks.com/help/install/install-products.html) (2022b or higher)
- [Simulink](https://www.mathworks.com/help/install/install-products.html) (2022b or higher)
- [Git](https://git-scm.com/downloads)
  - Used for maintaining and updating the jet vanes sims repo

## How to Run

Always run `setup.m` in the root directory first before running any other simulation files in order to properly set up paths and the global simulation environment.

Current simulations:
- `run_in_flight_ekf.m`: Runs the in flight ekf
- `run_ground_ekf.m`: Runs the ground ekf
- `run_open_loop.m`: Runs the open loop 6DOF truth trajectory 

## Contributing

First, fork the jet-vanes-simulations repo so that you can make changes independently. 

Clone your fork locally, do not directly clone the repo.

```
git clone https://github.gatech.edu/<your-github-username>/jet-vanes-simulations.git
```

Whenever updates are made to the upstream repo (the official [jet-vanes-simulations](https://github.gatech.edu/gnc/jet-vanes-simulations) repo), you will want to update your local fork to the most recent changes. Make sure your local fork is fully up to date before making any pushes:
```
cd jet-vanes-simulations
git checkout master
git pull https://github.gatech.edu/gnc/jet-vanes-simulations.git master
```

To update your changes to your fork:
```
git add <file(s) that you edited>
git commit -m '<description of the updates>'
git push
```