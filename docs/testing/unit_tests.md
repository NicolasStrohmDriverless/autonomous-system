# Unit Tests

We use Test frameworks depending on the language the package is written in. For C++ we utilize [Google Test](https://google.github.io/googletest/) and in the case of a Python package we resort to [Pytest](https://docs.pytest.org/)

## Modifications

In order to implement tests and make them visible during the build process we need to add a few lines of code to package files.

=== "CMake C++"
    ```cmake title="CMakeLists.txt"
    if(BUILD_TESTING)
        find_package(ament_cmake_gtest REQUIRED)

        ament_add_gtest(test_${PROJECT_NAME} <source_file> <test_file>)
        target_include_directories(test_${PROJECT_NAME} PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
        )
        ament_target_dependencies(test_${PROJECT_NAME}
            #dependencies of source and test files
        )
        target_link_libraries(test_${PROJECT_NAME} ${PCL_LIBRARIES})
    endif()
    ```
    ```xml title="package.xml"
    <test_depend>ament_cmake_gtest</test_depend>
    ```

=== "CMake Python"
    ```cmake title="CMakeLists.txt"
    if(BUILD_TESTING)
        find_package(ament_cmake_pytest REQUIRED)

        set(_pytest_tests
        #add your test files
        )
        foreach(_test_path ${_pytest_tests})
            get_filename_component(_test_name ${_test_path} NAME_WE)
            ament_add_pytest_test(${_test_name} ${_test_path}
            APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
            TIMEOUT 60
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            )
        endforeach()
    endif()
    ```
    ```xml title="package.xml"
    <test_depend>ament_cmake_pytest</test_depend>
    ```

## Implementation

All Unit tests will be located inside the folder ``test`` of your package. Go ahead and create it.

=== "CMake C++"
    For test files, we use the naming convention ``test_file_to_test.cpp``.

    After creation, we need to prepare the file for our tests.
    ```cpp title="test_file_to_test.cpp"
    #include <gtest/gtest.h>
    #include "file_to_test.hpp"
    //add any other dependencies

    class ClassToTestTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            //setup any object you might need for multiple test cases
        }
        void TearDown() override
        {
            //clean up your objects
        }
    };

    TEST_F(ClassToTestTest, Method_TestCase) {
        //your test logic
    }
    ```

=== "CMake Python"
    For test files we use the naming convention ``test_file_to_test.py``.

    After creation we need to prepare the file for our tests.
    ```py title="test_file_to_test.cpp"
    import pytest
    import file_to_test.py
    #add any other dependencies

    @pytest.fixture
    def setup_teardown():
        # Setup (if needed)
            #setup any object you might need for multiple test cases
        
        # Teardown (if needed)
            #clean up your objects
    
    @pytest.mark.usefixtures("setup_teardown")
    class TestClassToTest:
        
        def test_Method_TestCase(self, setup_teardown):
            #your test logic 
    ```

    !!! note
        Every test case has to start with ``test_`` in order to be picked up by pytest.

## Running the tests

After building your package again, you can run your tests.

```bash
colcon build --packages-select <your_package>
```

```bash
colcon test --ctest-args tests --packages-select <your_package>
```

!!! note 
    Package selection is optional.

In order to output the results of the test, use this command.

```
colcon test-result --all
```

!!! note
    If you want to have a look into the resulting XML-Files directly you can find them under ``build/<package_name>/test_results/<package_name>``.