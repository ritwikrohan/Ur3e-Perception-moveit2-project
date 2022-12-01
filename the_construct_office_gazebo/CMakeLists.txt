cmake_minimum_required(VERSION 3.8)
project(the_construct_office_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
# find_package(rmf_building_map_tools REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter, which checks for copyrights
  # uncomment the line when copyright and license are not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a Git repo)
  # uncomment the line when this package is not in a Git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


message(NOTICE "Starting Compilation...")

file(GLOB_RECURSE traffic_editor_paths "rmf_config/*.building.yaml")

foreach(path ${traffic_editor_paths})
  message(NOTICE "path: ${path}")

  # Get the output world name
  string(REGEX REPLACE "\\.[^.]*\.[^.]*$" "" no_extension_path ${path})
  string(REGEX MATCH "[^\/]+$" world_name  ${no_extension_path})

  set(map_path ${path})
  set(output_world_name ${world_name})
  set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/maps/${output_world_name})
  set(output_world_path ${output_dir}/${output_world_name}.world)
  set(output_model_dir ${output_dir}/models)

  message(NOTICE "map_path: ${map_path}")
  message(NOTICE "output_world_name: ${output_world_name}")
  message(NOTICE "output_dir: ${output_dir}")

  message(NOTICE "output_world_path: ${output_world_path}")
  message(NOTICE "output_model_dir: ${output_model_dir}")

  ##############################################################################
  # Generate Gz world and download Models
  ##############################################################################

  message(NOTICE "BUILDING WORLD FILE WITH COMMAND: ros2 run rmf_building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir}")
  message(NOTICE "....Downloading Models and Building to ${output_model_dir}")
  add_custom_command(
    OUTPUT ${output_world_path}
    COMMAND ros2 run rmf_building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir}
    COMMAND ros2 run rmf_building_map_tools building_map_model_downloader ${map_path} -f -e ${output_model_dir}
    DEPENDS ${map_path}
  )

  add_custom_target(generate_${output_world_name}_world ALL
    DEPENDS ${output_world_path}
  )

  message(NOTICE "....Downloading and Building Models to ${output_model_dir} >>>>> DONE")


  message(NOTICE "....BUILDING WORLD FILE...DONE")

  ##############################################################################
  # Generate the nav graphs
  ##############################################################################

  set(output_nav_graphs_dir ${output_dir}/nav_graphs/)
  set(output_nav_graphs_phony ${output_nav_graphs_dir}/phony)
  message(NOTICE "....Generating Lane Graphs in  ${output_nav_graphs_dir}")
  add_custom_command(
    OUTPUT ${output_nav_graphs_phony}
    COMMAND ros2 run rmf_building_map_tools building_map_generator nav ${map_path} ${output_nav_graphs_dir}
    DEPENDS ${map_path}
  )

  add_custom_target(generate_${output_world_name}_nav_graphs ALL
    DEPENDS ${output_nav_graphs_phony}
  )

  install(
    DIRECTORY ${output_dir}
    DESTINATION share/${PROJECT_NAME}/maps
  )

  message(NOTICE "....Generating Lane Graphs in  ${output_nav_graphs_dir}...DONE")

  ##############################################################################
  # Copy Generated Map to package with different name
  ##############################################################################

  	string(REGEX REPLACE "\\.[^.]*\.[^.]*$" "" no_extension_path ${path})


	if( "${output_world_name}" STREQUAL "starbots_complete")
		set(copyworld_phony ${no_extension_path}/phony)
		message(NOTICE "....Copying World from ${output_world_path} ${no_extension_path}")
		add_custom_command(
			OUTPUT ${copyworld_phony}
			COMMAND cp -rf ${output_world_path} ${no_extension_path}_copy.world
			DEPENDS ${output_world_path}
		)

		add_custom_target(${output_world_name}_generate_freezeworld ALL
			DEPENDS ${copyworld_phony}
		)
	else()
		message("Not Copying world file ${output_world_name}")
	endif()

  

endforeach()

install(DIRECTORY
  rmf_config
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS

  DESTINATION lib/${PROJECT_NAME}
)