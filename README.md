# drone
## Links of interest
Tired of using a password or token to push your commits ? Consider [seting up an ssh key](https://blog.corsego.com/aws-cloud9-github-ssh).
## License
This software is licensed under [MIT](LICENSE)
## Getting LSP support
Run `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` instead, then link the `compile_commands.json` inside the build folder to where your source file is.

# FOR COMP!!! (Do this in this order)
## Disclaimer
I have not tested the code on the actual system, please try it before comp day.
## On the ground computer (pre-mission)
This will start a node that captures images sent by the drone and save them under `~/mapping/images` on the ground computer
Run `start_ground_node.sh`

## On the drone
This will start the node that publishes images on a topic.
Run `start_drone_node.sh`

### Troubleshooting
The drone's ip address may not be the one I thought it would be, thus leading to the connection never being established. The 2 nodes that are used during the mission are under `drone/drone_ws/src/camera/camera/` so take a look at those
## QGroundControl
This will make a `.plan` file to be uploaded in QGroundControl. 
1. Figure out which runway you're on, either 1 or 2.
2. Modify `mission_setup_script/comp_mutables.json` to put in the appropriate lap waypoints.
3. Run `python3 mission_setup_script/mission_generator.py <runway_number>`, where runway number is either 1 or 2.
4. In QGroundControl upload the `.plan` file generated (will be under `mission_setup_script`)
5. Verify that all is in place, add waypoints if any trajectories between waypoints lie outside the mission boundary!!!
6. For all survey items (last 2 items at the time of writting), move the angle slider to generate the waypoints, also tinker with the other parameters if need be.
7. Add takeoff and landing commands if need be.
8. Fix things that seem odd, may wanna move cruise speed and such 
## On the ground computer (post-mission)
Supposedly, all images taken during the mission are now in `~/mapping/images/`
Run `sudo docker run -ti --rm -v ~/mapping:/datasets/code opendronemap/odm --project-path /datasets`
This will stitch images together and generate some other folders under `~/mapping/` (I think it's a file ending in .tiff but I am not sure)
