if(BUILD_ROS_INTERFACE)

  find_package(rostest REQUIRED)
  add_rostest(ros/subscribers.launch.test)

  set(MODULES agimus_controller)

  add_python_unit_test(test_build_model test/test_build_model.py)
  add_python_unit_test(test_run_mains test/test_run_mains.py)
  add_python_unit_test(test_servers test/test_servers.py)
  add_python_unit_test(test_scenes test/test_scenes.py)

endif()
