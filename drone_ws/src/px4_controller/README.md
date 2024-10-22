# PX4 controller package
Provides interface for monitoring and controlling the drone.
## Dependency
Requires MAVSDK. 
Download the appropriate `.deb` file throught the [release link](https://github.com/mavlink/MAVSDK/releases/tag/v2.12.11). Once you download the file, install it through `dpkg -i <file>.deb`.
## For better LSP support
Compile with the `colcon ... --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` to generate a file that helps the LSP. Make sure to link to that file from the build directory to the src directory of the code.
