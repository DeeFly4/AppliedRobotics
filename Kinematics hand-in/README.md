# Kinematics hand-in

This requires [Peter Corke's Robotics Toolbox  for Python](https://github.com/petercorke/robotics-toolbox-python) or [MATLAB](https://petercorke.com/toolboxes/robotics-toolbox/). You might have to downgrade the Python version to ensure compatibility with the robotics toolbox. Check the repo. The robotics toolbox includes tons of packages like numpy and matplotlib. DO NOT install these on your own to ensure compatibility.

```console
conda create -n robotics
conda activate robotics
conda install python=3.11
conda install conda-forge::mkl-service
conda install conda-forge::roboticstoolbox-python
```