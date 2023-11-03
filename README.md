# Bicopter-Vision-Control
This repo contains code for blob detection running on Nicla Vision used for target and goal tracking.

## OpenMV IDE Package Import
OpenMV IDE will always try to look for imported packages in the `External USB Drive` of the board connected (Nicla Vison Storage), **regardless of whether the code is running on the board or PC**. All self-defined libraries should be placed inside the `USB Drive`. 