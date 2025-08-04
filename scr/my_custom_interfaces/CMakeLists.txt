cmake_minimum_required(VERSION 3.8)
project(my_custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetCoordinate.msg"  # don't add (,) coma here
  "srv/MyService.srv"  # add your service file here
  # you can add multiple message files here
  DEPENDENCIES geometry_msgs  # add dependencies here if needed 
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
