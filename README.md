# drone
## Links of interest
Tired of using a password or token to push your commits ? Consider [seting up an ssh key](https://blog.corsego.com/aws-cloud9-github-ssh).
## License
This software is licensed under [MIT](LICENSE)
## Getting LSP support
Run `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` instead, then link the `compile_commands.json` inside the build folder to where your source file is.

# FOR COMP!!!
## On the drone
This will start the node that publishes images on a topic.
Run `start_drone_node.sh`
## On the ground computer
This needs to be done still heh (Sorry if you're reading this at comp :( )
## QGroundControl
1. Figure out which runway you're on, either 1 or 2.
2. Modify `mission_setup_script/comp_mutables.json` to put in the appropriate lap waypoints.
3. Run `python3 mission_setup_script/mission_generator.py <runway_number>`, where runway number is either 1 or 2.
4. In QGroundControl upload the `.plan` file generated (will be under `mission_setup_script`)
5. Verify that all is in place, add waypoints if any trajectories between waypoints lie outside the mission boundary!!!
6. For all survey items (last 2 items at the time of writting), move the angle slider to generate the waypoints.
7. Add takeoff and landing commands if need be.
