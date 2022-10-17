# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "SmartCar-Tools/map_tools/modules/grid_map_generator/CMakeFiles/pcd_grid_divider_autogen.dir/AutogenUsed.txt"
  "SmartCar-Tools/map_tools/modules/grid_map_generator/CMakeFiles/pcd_grid_divider_autogen.dir/ParseCache.txt"
  "SmartCar-Tools/map_tools/modules/grid_map_generator/pcd_grid_divider_autogen"
  "SmartCar-Tools/map_tools/modules/trajectory_generator/CMakeFiles/traj_generator_autogen.dir/AutogenUsed.txt"
  "SmartCar-Tools/map_tools/modules/trajectory_generator/CMakeFiles/traj_generator_autogen.dir/ParseCache.txt"
  "SmartCar-Tools/map_tools/modules/trajectory_generator/traj_generator_autogen"
  )
endif()
