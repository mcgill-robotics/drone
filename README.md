# drone
## Links of interest
Tired of using a password or token to push your commits ? Consider [seting up an ssh key](https://blog.corsego.com/aws-cloud9-github-ssh).
## License
This software is licensed under [MIT](LICENSE)
## Getting LSP support
Run `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` instead, then link the `compile_commands.json` inside the build folder to where your source file is.
