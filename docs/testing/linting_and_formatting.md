# Linting and Formatting

ROS2 supports some out of the box linting. Linting and formatting are essential practices in software development aimed at improving code quality, readability, and maintainability. By integrating linting and formatting tools into development processes, we can enhance code quality, reduce errors, and maintain a consistent codebase.

## Linting

- **Purpose:** <br> Linting is the process of automatically checking code for errors, potential bugs, stylistic inconsistencies, and adherence to coding standards.
- **Functionality:**
    - Identifies syntax errors and common mistakes.
    - Enforces coding standards and best practices.
    - Can be integrated into development workflows to provide real-time feedback.
- **Example:** <br> Highlighting unused variables or incorrect use of data types.

## Formatting

- **Purpose:** <br> Formatting ensures that code follows a consistent style, making it easier to read and maintain.
- **Functionality:**
    - Automatically arranges code according to predefined style rules.
    - Standardizes indentation, spacing, and line breaks.
    - Helps teams adhere to a unified code style.
- **Example:** <br> Ensuring all indentation is 4 spaces, or converting single quotes to double quotes in strings.

## Modification 

In order to implement format and lint tests, we need to add a few lines of code to package files.

=== "CMake C++"
    ```cmake title="CMakeLists.txt"
    if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
    endif()
    ```
    ```xml title="package.xml"
    <test_depend>ament_lint_auto</test_depend>
    <test_depend>ament_cmake_lint_cmake</test_depend>
    <test_depend>ament_cmake_uncrustify</test_depend>
    <test_depend>ament_cmake_xmllint</test_depend>
    ```

=== "CMake Python"
    ```cmake title="CMakeLists.txt"
    if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
    endif()
    ```
    ```xml title="package.xml"
    <test_depend>ament_lint_auto</test_depend>
    <test_depend>ament_cmake_flake8</test_depend>
    <test_depend>ament_cmake_xmllint</test_depend>
    <test_depend>ament_cmake_lint_cmake</test_depend>
    ```