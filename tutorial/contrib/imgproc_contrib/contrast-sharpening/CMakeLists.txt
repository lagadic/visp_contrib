project(tutorial-contrast-sharpening-contrib)

cmake_minimum_required(VERSION 3.0)

find_package(VISP REQUIRED visp_core visp_io visp_gui visp_imgproc_contrib)

# set the list of source files
set(tutorial_cpp
  tutorial-contrast-sharpening-contrib.cpp)

list(APPEND tutorial_data "${CMAKE_CURRENT_SOURCE_DIR}/Crayfish-low-contrast.ppm")

foreach(cpp ${tutorial_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "tutorials")
  endif()
endforeach()

# Copy the data files to the same location than the target
foreach(data ${tutorial_data})
  visp_copy_data(tutorial-contrast-sharpening-contrib.cpp ${data})
endforeach()
