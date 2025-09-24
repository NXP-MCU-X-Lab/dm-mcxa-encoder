# Add set(CONFIG_USE_device_startup true) in config.cmake to use this component

include_guard(GLOBAL)
message("${CMAKE_CURRENT_LIST_FILE} component is included.")

      if(CONFIG_TOOLCHAIN STREQUAL mcux)
          add_config_file(${CMAKE_CURRENT_LIST_DIR}/mcuxpresso/startup_MCXA344.c "" device_startup.MCXA344)
          add_config_file(${CMAKE_CURRENT_LIST_DIR}/startup_MCXA344.cpp "" device_startup.MCXA344)
        endif()

        if((CONFIG_TOOLCHAIN STREQUAL iar OR CONFIG_TOOLCHAIN STREQUAL armgcc OR CONFIG_TOOLCHAIN STREQUAL mdk))
          add_config_file(${CMAKE_CURRENT_LIST_DIR}/startup_MCXA344.c "" device_startup.MCXA344)
        endif()

  

