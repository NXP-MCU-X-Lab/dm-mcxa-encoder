# Add set(CONFIG_USE_middleware_freertos-kernel_config true) in config.cmake to use this component

include_guard(GLOBAL)
message("${CMAKE_CURRENT_LIST_FILE} component is included.")

            add_config_file(${CMAKE_CURRENT_LIST_DIR}/template/FreeRTOSConfig.h ${CMAKE_CURRENT_LIST_DIR}/template middleware_freertos-kernel_config)
    
  

