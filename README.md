# Geometry Common

Common data structures, utilities and modules for geometric tasks.

## Documentation

We use [Doxygen](https://www.doxygen.nl/index.html) for code documentation. Building the documentation is disabled by default.

To build the code documentation, you have to install Doxygen, using `sudo apt install doxygen`
1. Documentation can be built using the flag `-DBUILD_DOC=ON`. For example: `catkin build geometry_common -DBUILD_DOC=ON`
2. The documentation will be generated in the directory `catkin_ws/build/geometry_common/docs/html/index.html`
