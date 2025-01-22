# Onboard software workspace
## Install all dependencies
First, you need to manually install MAVProxy. Refer to the `README.md` inside the `./src/px4_controller` directory for instructions.
Then, you will need to install the remaining dependencies, run `rosdep install --from-paths src -y --ignore-src`. If this throws an error and asks you to run other commands beforehand, please do so.
