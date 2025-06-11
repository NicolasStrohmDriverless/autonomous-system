# How to create a ROS2 package

## Structure
Depending on what language you want to use, ROS2 packages are always structured the same.

=== "CMake"

    - ``CMakeLists.txt`` file that describes how to build the code within the package
    - ``include`` directory containing the public headers for the package
    - ``package.xml`` file containing meta information about the package
    - ``src`` directory containing the source code for the package

=== "Python"

    - ``package.xml`` file containing meta information about the package
    - ``resource/<package_name>`` marker file for the package
    - ``setup.cfg`` is required when a package has executables, so ros2 run can find them
    - ``setup.py`` containing instructions for how to install the package
    - ``<package_name>`` - a directory with the same name as your package, used by ROS 2 tools to find your package, contains ``__init__.py``

A resulting folder structure could be following

=== "CMake"
    
    ```
    .
    ├── my_package/
    │   ├── CMakeLists.txt
    │   ├── include/
    │   ├── package.xml
    │   └── src/
    ```

=== "Python"

    ```
    ├── my_package/
    │   ├── package.xml
    │   ├── resource/my_package
    │   ├── setup.cfg
    │   ├── setup.py
    │   └── my_package/
    ```

## Creating a Package

To create your package, you need to follow this basic Syntax.

=== "CMake"
    ```bash
    ros2 pkg create --build-type ament_cmake --destination-directory src/subsystems --maintainer-email <your-strohmo-mail> --maintainer-name "<your-name>" --node-name <package_name> <package_name>
    ```

=== "Python"
    ```bash
    ros2 pkg create --build-type ament_python --destination-directory src/subsystems --maintainer-email <your-strohmo-mail> --maintainer-name "<your-name>" --node-name <package_name> <package_name>
    ```

!!! note "Advanced"
    If you have your dependencies predefined, you can already append them to this command with ``--dependencies <your-dependencies>``.

??? note "List of Parameters"
    <table>
        <tr>
            <th>Option</th>
            <th>Functionality</th>
        </tr>
        <tr>
            <td>``--package-format {2,3}, --package_format {2,3}``</td>
            <td>The package.xml format.</td>
        </tr>
        <tr>
            <td>``--description DESCRIPTION``</td>
            <td>The description given in the package.xml</td>
        </tr>
        <tr>
            <td>``--license LICENSE``</td>
            <td>The license attached to this package; this can be an arbitrary string, but a LICENSE file will only be generated if it is one of the supported licenses (pass '?' to get a list)</td>
        </tr>
        <tr>
            <td>`` --destination-directory DESTINATION_DIRECTORY``</td>
            <td>Directory where to create the package directory</td>
        </tr>
        <tr>
            <td>`` --build-type {cmake,ament_cmake,ament_python}``</td>
            <td>The build type to process the package with</td>
        </tr>
        <tr>
            <td>``--maintainer-email MAINTAINER_EMAIL``</td>
            <td>email address of the maintainer of this package</td>
        </tr><tr>
            <td>``--maintainer-name MAINTAINER_NAME``</td>
            <td>name of the maintainer of this package</td>
        </tr><tr>
            <td>``--node-name NODE_NAME``</td>
            <td>name of the empty executable</td>
        </tr><tr>
            <td>``--library-name LIBRARY_NAME``</td>
            <td>name of the empty library</td>
        </tr>
    </table>