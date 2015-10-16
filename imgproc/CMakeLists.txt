vp_add_module(imgproc visp_core)
vp_glob_module_sources()
vp_module_include_directories()
vp_create_module()

#if (CMAKE_VERSION VERSION_LESS 2.8.12)
#  vp_add_tests(
#    SOURCES_EXCLUDE
#    DEPENDS_ON visp_imgproc)
#else()
#  vp_add_tests(DEPENDS_ON visp_imgproc)
#endif()
