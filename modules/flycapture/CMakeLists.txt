#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2015 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See http://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# ViSP configuration file.
#
# Authors:
# Fabien Spindler
#
#############################################################################

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Looking for module dependencies 
if(NOT FLYCAPTURE_FOUND)
  VP_OPTION(USE_FLYCAPTURE FlyCapture "" "Include FlyCapture SDK support for PointGrey cameras" "" ON)
endif()

VP_SET(VISP_HAVE_FLYCAPTURE        TRUE IF (BUILD_MODULE_visp_flycapture AND USE_FLYCAPTURE))

# Add optional 3rd parties
set(opt_incs "")
set(opt_libs "")

# camera devices: Point Grey FlyCapture SDK
if(USE_FLYCAPTURE)
  list(APPEND opt_incs ${FLYCAPTURE_INCLUDE_DIRS})
  list(APPEND opt_libs ${FLYCAPTURE_LIBRARIES})
endif()

vp_add_module(flycapture visp_core)
vp_glob_module_sources()
vp_module_include_directories(${opt_incs})
vp_create_module(${opt_libs})

vp_add_config_file("cmake/template/vpConfigFlycapture.h.in")
vp_add_tests(DEPENDS_ON visp_gui)

