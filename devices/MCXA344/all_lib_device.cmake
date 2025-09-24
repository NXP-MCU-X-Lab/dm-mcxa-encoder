# Copy variable into project config.cmake to use software component
#set.board.frdmmcxa344
#  # description: Board_project_template frdmmcxa344
#  set(CONFIG_USE_board_project_template true)

#set.CMSIS_DSP_Lib
#  # description: CMSIS-DSP Library Header
#  set(CONFIG_USE_CMSIS_DSP_Include true)

#  # description: CMSIS-DSP Library
#  set(CONFIG_USE_CMSIS_DSP_Source true)

#set.CMSIS
#  # description: Device interrupt controller interface
#  set(CONFIG_USE_CMSIS_Device_API_OSTick true)

#  # description: CMSIS-RTOS API for Cortex-M, SC000, and SC300
#  set(CONFIG_USE_CMSIS_Device_API_RTOS2 true)

#  # description: Access to #include Driver_CAN.h file for custom implementation
#  set(CONFIG_USE_CMSIS_Driver_Include_CAN true)

#  # description: Access to #include Driver_ETH.h file for custom implementation
#  set(CONFIG_USE_CMSIS_Driver_Include_Ethernet true)

#  # description: Access to #include Driver_ETH_MAC.h file for custom implementation
#  set(CONFIG_USE_CMSIS_Driver_Include_Ethernet_MAC true)

#  # description: Access to #include Driver_ETH_PHY.h file for custom implementation
#  set(CONFIG_USE_CMSIS_Driver_Include_Ethernet_PHY true)

#  # description: Access to #include Driver_Flash.h file for custom implementation
#  set(CONFIG_USE_CMSIS_Driver_Include_Flash true)

#  # description: Access to #include Driver_GPIO.h file for custom implementation
#  set(CONFIG_USE_CMSIS_Driver_Include_GPIO true)

#  # description: Access to #include Driver_I2C.h file for custom implementation
#  set(CONFIG_USE_CMSIS_Driver_Include_I2C true)

#  # description: Access to #include Driver_MCI.h file for custom implementation
#  set(CONFIG_USE_CMSIS_Driver_Include_MCI true)

#  # description: Access to #include Driver_NAND.h file for custom implementation
#  set(CONFIG_USE_CMSIS_Driver_Include_NAND true)

#  # description: Access to #include Driver_SAI.h file for custom implementation
#  set(CONFIG_USE_CMSIS_Driver_Include_SAI true)

#  # description: Access to #include Driver_SPI.h file for custom implementation
#  set(CONFIG_USE_CMSIS_Driver_Include_SPI true)

#  # description: Access to #include Driver_USART.h file for custom implementation
#  set(CONFIG_USE_CMSIS_Driver_Include_USART true)

#  # description: Access to #include Driver_USBD.h file for custom implementation
#  set(CONFIG_USE_CMSIS_Driver_Include_USB_Device true)

#  # description: Access to #include Driver_USBH.h file for custom implementation
#  set(CONFIG_USE_CMSIS_Driver_Include_USB_Host true)

#  # description: Access to #include Driver_WiFi.h file
#  set(CONFIG_USE_CMSIS_Driver_Include_WiFi true)

#  # description: CMSIS-NN Library
#  set(CONFIG_USE_CMSIS_NN_Source true)

#  # description: CMSIS-CORE for Cortex-M, ARMv8-M, ARMv8.1-M
#  set(CONFIG_USE_CMSIS_Include_core_cm true)

#  # description: CMSIS-RTOS2 RTX5 for Cortex-M, SC000, C300 and Armv8-M (Library)
#  set(CONFIG_USE_CMSIS_RTOS2_RTX true)

#  # description: CMSIS-RTOS2 RTX5 for Cortex-M, SC000, C300 and Armv8-M (Library)
#  set(CONFIG_USE_CMSIS_RTOS2_RTX_LIB true)

#set.device.MCXA344
#  # description: EDMA SOC Driver
#  set(CONFIG_USE_driver_edma_soc true)

#  # description: Inputmux_connections Driver
#  set(CONFIG_USE_driver_inputmux_connections true)

#  # description: Reset Driver
#  set(CONFIG_USE_driver_reset true)

#  # description: TRDC SOC Driver
#  set(CONFIG_USE_driver_trdc_soc true)

#  # description: Utilities which is needed for particular toolchain like the SBRK function required to address limitation between HEAP and STACK in GCC toolchain library.
#  set(CONFIG_USE_utilities_misc_utilities true)

#  # description: Used to include slave core binary into master core binary.
#  set(CONFIG_USE_utility_incbin true)

#  # description: Used to format convertion
#  set(CONFIG_USE_utility_format true)

#  # description: common Driver
#  set(CONFIG_USE_driver_common true)

#  # description: Driver ektf2k
#  set(CONFIG_USE_driver_ektf2k true)

#  # description: Touch panel controller FT3267 driver
#  set(CONFIG_USE_driver_ft3267 true)

#  # description: Driver ft5406
#  set(CONFIG_USE_driver_ft5406 true)

#  # description: Driver gt911
#  set(CONFIG_USE_driver_gt911 true)

#  # description: Driver tma525b
#  set(CONFIG_USE_driver_tma525b true)

#  # description: Component serial_manager_swo
#  set(CONFIG_USE_component_serial_manager_swo true)

#  # description: Component serial_manager_virtual
#  set(CONFIG_USE_component_serial_manager_virtual true)

#  # description: RTT template configuration
#  set(CONFIG_USE_driver_rtt_template true)

#  # description: Component reset_adapter
#  set(CONFIG_USE_component_reset_adapter true)

#  # description: Component panic
#  set(CONFIG_USE_component_panic true)

#  # description: Devices_project_template MCXA344
#  set(CONFIG_USE_device_project_template true)

#  # description: Device MCXA344_cmsis
#  set(CONFIG_USE_device_CMSIS true)

#  # description: Device MCXA344_system
#  set(CONFIG_USE_device_system true)

#  # description: Device MCXA344_startup
#  set(CONFIG_USE_device_startup true)

#  # description: Rte_device
#  set(CONFIG_USE_device_RTE true)

#  # description: Clock Driver
#  set(CONFIG_USE_driver_clock true)

#  # description: ROMAPI Driver
#  set(CONFIG_USE_driver_romapi true)

#  # description: Utility str
#  set(CONFIG_USE_utility_str true)

#  # description: Utility debug_console_lite
#  set(CONFIG_USE_utility_debug_console_lite true)

#  # description: Utility debug_console
#  set(CONFIG_USE_utility_debug_console true)

#  # description: Utility debug_console Template Config
#  set(CONFIG_USE_utility_debug_console_template_config true)

#  # description: Utility assert
#  set(CONFIG_USE_utility_assert true)

#  # description: Utility assert_lite
#  set(CONFIG_USE_utility_assert_lite true)

#  # description: LPUART Freertos Driver
#  set(CONFIG_USE_driver_lpuart_freertos true)

#  # description: LPSPI FreeRTOS Driver
#  set(CONFIG_USE_driver_lpspi_freertos true)

#  # description: LPI2C FreeRTOS Driver
#  set(CONFIG_USE_driver_lpi2c_freertos true)

#  # description: WWDT Driver
#  set(CONFIG_USE_driver_wwdt true)

#  # description: WUU Driver
#  set(CONFIG_USE_driver_wuu true)

#  # description: WAKETIMER Driver
#  set(CONFIG_USE_driver_waketimer true)

#  # description: UTICK Driver
#  set(CONFIG_USE_driver_utick true)

#  # description: TRDC Driver
#  set(CONFIG_USE_driver_trdc_1 true)

#  # description: PWM Driver
#  set(CONFIG_USE_driver_pwm true)

#  # description: PORT Driver
#  set(CONFIG_USE_driver_port true)

#  # description: RTC Driver
#  set(CONFIG_USE_driver_rtc true)

#  # description: OSTimer Driver
#  set(CONFIG_USE_driver_ostimer true)

#  # description: OPAMP Driver
#  set(CONFIG_USE_driver_opamp_fast true)

#  # description: MCX VBAT Driver
#  set(CONFIG_USE_driver_mcx_vbat true)

#  # description: MCX SPC Driver
#  set(CONFIG_USE_driver_mcx_spc true)

#  # description: MCX_CMC Driver
#  set(CONFIG_USE_driver_mcx_cmc true)

#  # description: MAU Driver
#  set(CONFIG_USE_driver_mau true)

#  # description: LPUART Driver
#  set(CONFIG_USE_driver_lpuart true)

#  # description: LPUART Driver
#  set(CONFIG_USE_driver_lpuart_edma true)

#  # description: LPUART CMSIS Driver
#  set(CONFIG_USE_driver_cmsis_lpuart true)

#  # description: LPTMR Driver
#  set(CONFIG_USE_driver_lptmr true)

#  # description: LPSPI Driver
#  set(CONFIG_USE_driver_lpspi true)

#  # description: LPSPI Driver
#  set(CONFIG_USE_driver_lpspi_edma true)

#  # description: LPSPI CMSIS Driver
#  set(CONFIG_USE_driver_cmsis_lpspi true)

#  # description: LPI2C Driver
#  set(CONFIG_USE_driver_lpi2c true)

#  # description: LPI2C Driver
#  set(CONFIG_USE_driver_lpi2c_edma true)

#  # description: LPI2C CMSIS Driver
#  set(CONFIG_USE_driver_cmsis_lpi2c true)

#  # description: LPCMP Driver
#  set(CONFIG_USE_driver_lpcmp true)

#  # description: lpc_freqme Driver
#  set(CONFIG_USE_driver_lpc_freqme true)

#  # description: LPADC Driver
#  set(CONFIG_USE_driver_lpadc true)

#  # description: INPUTMUX Driver
#  set(CONFIG_USE_driver_inputmux true)

#  # description: GPIO Driver
#  set(CONFIG_USE_driver_gpio true)

#  # description: GPIO CMSIS Driver
#  set(CONFIG_USE_driver_cmsis_gpio true)

#  # description: GLIKEY Driver
#  set(CONFIG_USE_driver_glikey true)

#  # description: FLEXCAN Driver
#  set(CONFIG_USE_driver_flexcan true)

#  # description: FLEXCAN Driver
#  set(CONFIG_USE_driver_flexcan_edma true)

#  # description: ERM Driver
#  set(CONFIG_USE_driver_erm true)

#  # description: EQDC Driver
#  set(CONFIG_USE_driver_eqdc true)

#  # description: EIM Driver
#  set(CONFIG_USE_driver_eim true)

#  # description: EDMA Driver
#  set(CONFIG_USE_driver_edma4 true)

#  # description: CTimer Driver
#  set(CONFIG_USE_driver_ctimer true)

#  # description: CRC Driver
#  set(CONFIG_USE_driver_crc true)

#  # description: cdog Driver
#  set(CONFIG_USE_driver_cdog true)

#  # description: CACHE Driver
#  set(CONFIG_USE_driver_cache_lpcac_n4a_mcxn true)

#  # description: AOI Driver
#  set(CONFIG_USE_driver_aoi true)

#  # description: Driver camera-common
#  set(CONFIG_USE_driver_camera-common true)

#  # description: Driver camera-device-ap1302
#  set(CONFIG_USE_driver_camera-device-ap1302 true)

#  # description: Driver camera-device-common
#  set(CONFIG_USE_driver_camera-device-common true)

#  # description: Driver camera-device-max9286
#  set(CONFIG_USE_driver_camera-device-max9286 true)

#  # description: Driver camera-device-mt9m114
#  set(CONFIG_USE_driver_camera-device-mt9m114 true)

#  # description: Driver camera-device-ov5640
#  set(CONFIG_USE_driver_camera-device-ov5640 true)

#  # description: Driver camera-device-ov7670
#  set(CONFIG_USE_driver_camera-device-ov7670 true)

#  # description: Driver camera-device-ov7725
#  set(CONFIG_USE_driver_camera-device-ov7725 true)

#  # description: Driver camera-device-sccb
#  set(CONFIG_USE_driver_camera-device-sccb true)

#  # description: Driver camera-receiver-common
#  set(CONFIG_USE_driver_camera-receiver-common true)

#  # description: Driver dbi
#  set(CONFIG_USE_driver_dbi true)

#  # description: Driver dc-fb-common
#  set(CONFIG_USE_driver_dc-fb-common true)

#  # description: Driver dc-fb-dbi
#  set(CONFIG_USE_driver_dc-fb-dbi true)

#  # description: Driver dc-fb-ssd1963
#  set(CONFIG_USE_driver_dc-fb-ssd1963 true)

#  # description: Driver display-adv7535
#  set(CONFIG_USE_driver_display-adv7535 true)

#  # description: Driver display-sn65dsi83
#  set(CONFIG_USE_driver_display-sn65dsi83 true)

#  # description: Driver display-common
#  set(CONFIG_USE_driver_display-common true)

#  # description: Driver display-it6161
#  set(CONFIG_USE_driver_display-it6161 true)

#  # description: Driver display-it6263
#  set(CONFIG_USE_driver_display-it6263 true)

#  # description: Driver fbdev
#  set(CONFIG_USE_driver_fbdev true)

#  # description: Driver video-common
#  set(CONFIG_USE_driver_video-common true)

#  # description: Driver video-i2c
#  set(CONFIG_USE_driver_video-i2c true)

#  # description: Component lpuart_adapter
#  set(CONFIG_USE_component_lpuart_adapter true)

#  # description: Driver ft5406_rt
#  set(CONFIG_USE_driver_ft5406_rt true)

#  # description: Driver ft6x06
#  set(CONFIG_USE_driver_ft6x06 true)

#  # description: Component timer_manager
#  set(CONFIG_USE_component_timer_manager true)

#  # description: Component ctimer_adapter
#  set(CONFIG_USE_component_ctimer_adapter true)

#  # description: Component lptmr_adapter
#  set(CONFIG_USE_component_lptmr_adapter true)

#  # description: Component ostimer_adapter
#  set(CONFIG_USE_component_ostimer_adapter true)

#  # description: Component ctimer time stamp adapter
#  set(CONFIG_USE_component_ctimer_time_stamp_adapter true)

#  # description: Component lptmr time stamp adapter
#  set(CONFIG_USE_component_lptmr_time_stamp_adapter true)

#  # description: Component ostimer time stamp adapter
#  set(CONFIG_USE_component_ostimer_time_stamp_adapter true)

#  # description: Component lpspi_adapter
#  set(CONFIG_USE_component_lpspi_adapter true)

#  # description: Utility shell
#  set(CONFIG_USE_utility_shell true)

#  # description: Component serial_manager
#  set(CONFIG_USE_component_serial_manager true)

#  # description: Component serial_manager_spi
#  set(CONFIG_USE_component_serial_manager_spi true)

#  # description: Component serial_manager_uart
#  set(CONFIG_USE_component_serial_manager_uart true)

#  # description: Driver fxas21002cq
#  set(CONFIG_USE_driver_fxas21002cq true)

#  # description: Driver fxls8974cf
#  set(CONFIG_USE_driver_fxls8974cf true)

#  # description: Driver fxos8700cq
#  set(CONFIG_USE_driver_fxos8700cq true)

#  # description: Driver htu21d
#  set(CONFIG_USE_driver_htu21d true)

#  # description: Driver icm42688p
#  set(CONFIG_USE_driver_icm42688p true)

#  # description: Driver lsm6dso
#  set(CONFIG_USE_driver_lsm6dso true)

#  # description: Driver max30101
#  set(CONFIG_USE_driver_max30101 true)

#  # description: Driver mma8451q
#  set(CONFIG_USE_driver_mma8451q true)

#  # description: Driver mma8652fc
#  set(CONFIG_USE_driver_mma8652fc true)

#  # description: Driver nmh1000
#  set(CONFIG_USE_driver_nmh1000 true)

#  # description: Driver p3t1755
#  set(CONFIG_USE_driver_p3t1755 true)

#  # description: Driver tsl2561
#  set(CONFIG_USE_driver_tsl2561 true)

#  # description: SEGGER Real Time Transfer(RTT)
#  set(CONFIG_USE_driver_rtt true)

#  # description: Component messaging
#  set(CONFIG_USE_component_messaging true)

#  # description: Component mem_manager
#  set(CONFIG_USE_component_mem_manager true)

#  # description: Component mem_manager_legacy
#  set(CONFIG_USE_component_mem_manager_legacy true)

#  # description: Component mem_manager_freertos
#  set(CONFIG_USE_component_mem_manager_freertos true)

#  # description: Component mem_manager_light
#  set(CONFIG_USE_component_mem_manager_light true)

#  # description: Component lists
#  set(CONFIG_USE_component_lists true)

#  # description: Component led
#  set(CONFIG_USE_component_led true)

#  # description: Component flash_adapter
#  set(CONFIG_USE_component_flash_adapter true)

#  # description: Component mcxw_flash_adapter
#  set(CONFIG_USE_component_mcxw_flash_adapter true)

#  # description: Component lpi2c_adapter
#  set(CONFIG_USE_component_lpi2c_adapter true)

#  # description: Component i2c_adapter_interface
#  set(CONFIG_USE_component_i2c_adapter_interface true)

#  # description: Component i2c_mux_pca954x
#  set(CONFIG_USE_component_i2c_mux_pca954x true)

#  # description: Component enable_pca9544
#  set(CONFIG_USE_component_enable_pca9544 true)

#  # description: Component enable_pca9548
#  set(CONFIG_USE_component_enable_pca9548 true)

#  # description: Component at_least_one_i2c_mux_device_enabled
#  set(CONFIG_USE_component_at_least_one_i2c_mux_device_enabled true)

#  # description: Component gpio_adapter
#  set(CONFIG_USE_component_gpio_adapter true)

#  # description: Component flash_nor_lpspi
#  set(CONFIG_USE_component_flash_nor_lpspi true)

#  # description: mflash onchip
#  set(CONFIG_USE_component_mflash_onchip true)

#  # description: mflash offchip
#  set(CONFIG_USE_component_mflash_offchip true)

#  # description: Driver ili9341
#  set(CONFIG_USE_driver_ili9341 true)

#  # description: Driver psp27801
#  set(CONFIG_USE_driver_psp27801 true)

#  # description: Driver ssd1963
#  set(CONFIG_USE_driver_ssd1963 true)

#  # description: Driver st7796s
#  set(CONFIG_USE_driver_st7796s true)

#  # description: Component button
#  set(CONFIG_USE_component_button true)

#set.middleware.littlefs
#  # description: littlefs
#  set(CONFIG_USE_middleware_littlefs true)

#  # description: littlefs mflash
#  set(CONFIG_USE_middleware_littlefs_mflash true)

#set.middleware.fatfs
#  # description: FatFs
#  set(CONFIG_USE_middleware_fatfs true)

#  # description: FatFs template MMC
#  set(CONFIG_USE_middleware_fatfs_template_mmc true)

#  # description: FatFs template NAND
#  set(CONFIG_USE_middleware_fatfs_template_nand true)

#  # description: FatFs template RAM
#  set(CONFIG_USE_middleware_fatfs_template_ram true)

#  # description: FatFs template SD
#  set(CONFIG_USE_middleware_fatfs_template_sd true)

#  # description: FatFs template SDSPI
#  set(CONFIG_USE_middleware_fatfs_template_sdspi true)

#  # description: FatFs template USB
#  set(CONFIG_USE_middleware_fatfs_template_usb true)

#  # description: FatFs_RAM
#  set(CONFIG_USE_middleware_fatfs_ram true)

#set.component.osa
#  # description: Component osa template config
#  set(CONFIG_USE_component_osa_template_config true)

#  # description: Component osa
#  set(CONFIG_USE_component_osa true)

#  # description: Component osa_bm
#  set(CONFIG_USE_component_osa_bm true)

#  # description: Component osa_free_rtos
#  set(CONFIG_USE_component_osa_free_rtos true)

#  # description: Component common_task
#  set(CONFIG_USE_component_common_task true)

#set.middleware.freertos-kernel
#  # description: FreeRTOS kernel
#  set(CONFIG_USE_middleware_freertos-kernel true)

#  # description: FreeRTOS cm33 non trustzone port
#  set(CONFIG_USE_middleware_freertos-kernel_cm33_non_trustzone true)

#  # description: FreeRTOS cm33 secure port
#  set(CONFIG_USE_middleware_freertos-kernel_cm33_trustzone_non_secure true)

#  # description: FreeRTOS Secure Context
#  set(CONFIG_USE_middleware_freertos-kernel_cm33_trustzone_secure true)

#  # description: FreeRTOS NXP extension
#  set(CONFIG_USE_middleware_freertos-kernel_extension true)

#  # description: FreeRTOS NXP Newlib Reentrant
#  set(CONFIG_USE_middleware_freertos-kernel_use_newlib_reentrant true)

#  # description: FreeRTOS NXP Picolibc TLS
#  set(CONFIG_USE_middleware_freertos-kernel_use_picolibc_tls true)

#  # description: FreeRTOS heap 1
#  set(CONFIG_USE_middleware_freertos-kernel_heap_1 true)

#  # description: FreeRTOS heap 2
#  set(CONFIG_USE_middleware_freertos-kernel_heap_2 true)

#  # description: FreeRTOS heap 3
#  set(CONFIG_USE_middleware_freertos-kernel_heap_3 true)

#  # description: FreeRTOS heap 4
#  set(CONFIG_USE_middleware_freertos-kernel_heap_4 true)

#  # description: FreeRTOS heap 5
#  set(CONFIG_USE_middleware_freertos-kernel_heap_5 true)

#  # description: old FreeRTOS MPU wrappers used before V10.6.0
#  set(CONFIG_USE_middleware_freertos-kernel_mpu_wrappers true)

#  # description: new V2 FreeRTOS MPU wrappers introduced in V10.6.0
#  set(CONFIG_USE_middleware_freertos-kernel_mpu_wrappers_v2 true)

#  # description: Template configuration file to be edited by user. Provides also memory allocator (heap_x), change variant if needed.
#  set(CONFIG_USE_middleware_freertos-kernel_config true)

#set.middleware.fmstr
#  # description: Common FreeMASTER driver code.
#  set(CONFIG_USE_middleware_fmstr true)

#  # description: FreeMASTER driver code for 32bit platforms, enabling communication between FreeMASTER or FreeMASTER Lite tools and MCU application. Supports Serial, CAN, USB and BDM/JTAG physical interface.
#  set(CONFIG_USE_middleware_fmstr_platform_gen32le true)

#  # description: FreeMASTER driver code for DSC platforms, enabling communication between FreeMASTER or FreeMASTER Lite tools and MCU application. Supports Serial, CAN, USB and BDM/JTAG physical interface.
#  set(CONFIG_USE_middleware_fmstr_platform_56f800e true)

#  # description: FreeMASTER driver code for S32 platform.
#  set(CONFIG_USE_middleware_fmstr_platform_s32 true)

#  # description: FreeMASTER driver code for Power Architecture 32bit platform.
#  set(CONFIG_USE_middleware_fmstr_platform_pa32 true)

#  # description: FreeMASTER driver code for S12Z platform.
#  set(CONFIG_USE_middleware_fmstr_platform_s12z true)

list(APPEND CMAKE_MODULE_PATH
  ${CMAKE_CURRENT_LIST_DIR}/.
  ${CMAKE_CURRENT_LIST_DIR}/../../CMSIS/Core/Include
  ${CMAKE_CURRENT_LIST_DIR}/../../CMSIS/DSP
  ${CMAKE_CURRENT_LIST_DIR}/../../CMSIS/Driver
  ${CMAKE_CURRENT_LIST_DIR}/../../CMSIS/NN
  ${CMAKE_CURRENT_LIST_DIR}/../../CMSIS/RTOS2
  ${CMAKE_CURRENT_LIST_DIR}/../../CMSIS/RTOS2/Include
  ${CMAKE_CURRENT_LIST_DIR}/../../boards/frdmmcxa344/project_template
  ${CMAKE_CURRENT_LIST_DIR}/../../components/button
  ${CMAKE_CURRENT_LIST_DIR}/../../components/common_task
  ${CMAKE_CURRENT_LIST_DIR}/../../components/display/ili9341
  ${CMAKE_CURRENT_LIST_DIR}/../../components/display/psp27801
  ${CMAKE_CURRENT_LIST_DIR}/../../components/display/ssd1963
  ${CMAKE_CURRENT_LIST_DIR}/../../components/display/st7796s
  ${CMAKE_CURRENT_LIST_DIR}/../../components/flash
  ${CMAKE_CURRENT_LIST_DIR}/../../components/flash/mflash
  ${CMAKE_CURRENT_LIST_DIR}/../../components/gpio
  ${CMAKE_CURRENT_LIST_DIR}/../../components/i2c
  ${CMAKE_CURRENT_LIST_DIR}/../../components/i2c/muxes
  ${CMAKE_CURRENT_LIST_DIR}/../../components/internal_flash
  ${CMAKE_CURRENT_LIST_DIR}/../../components/led
  ${CMAKE_CURRENT_LIST_DIR}/../../components/lists
  ${CMAKE_CURRENT_LIST_DIR}/../../components/mem_manager
  ${CMAKE_CURRENT_LIST_DIR}/../../components/messaging
  ${CMAKE_CURRENT_LIST_DIR}/../../components/osa
  ${CMAKE_CURRENT_LIST_DIR}/../../components/panic
  ${CMAKE_CURRENT_LIST_DIR}/../../components/reset
  ${CMAKE_CURRENT_LIST_DIR}/../../components/rtt
  ${CMAKE_CURRENT_LIST_DIR}/../../components/sensor/fxas21002cq
  ${CMAKE_CURRENT_LIST_DIR}/../../components/sensor/fxls8974cf
  ${CMAKE_CURRENT_LIST_DIR}/../../components/sensor/fxos8700cq
  ${CMAKE_CURRENT_LIST_DIR}/../../components/sensor/htu21d
  ${CMAKE_CURRENT_LIST_DIR}/../../components/sensor/icm42688p
  ${CMAKE_CURRENT_LIST_DIR}/../../components/sensor/lsm6dso
  ${CMAKE_CURRENT_LIST_DIR}/../../components/sensor/max30101
  ${CMAKE_CURRENT_LIST_DIR}/../../components/sensor/mma8451q
  ${CMAKE_CURRENT_LIST_DIR}/../../components/sensor/mma8652fc
  ${CMAKE_CURRENT_LIST_DIR}/../../components/sensor/nmh1000
  ${CMAKE_CURRENT_LIST_DIR}/../../components/sensor/p3t1755
  ${CMAKE_CURRENT_LIST_DIR}/../../components/sensor/tsl2561
  ${CMAKE_CURRENT_LIST_DIR}/../../components/serial_manager
  ${CMAKE_CURRENT_LIST_DIR}/../../components/spi
  ${CMAKE_CURRENT_LIST_DIR}/../../components/time_stamp
  ${CMAKE_CURRENT_LIST_DIR}/../../components/timer
  ${CMAKE_CURRENT_LIST_DIR}/../../components/timer_manager
  ${CMAKE_CURRENT_LIST_DIR}/../../components/touch/ektf2k
  ${CMAKE_CURRENT_LIST_DIR}/../../components/touch/ft3267
  ${CMAKE_CURRENT_LIST_DIR}/../../components/touch/ft5406
  ${CMAKE_CURRENT_LIST_DIR}/../../components/touch/ft5406_rt
  ${CMAKE_CURRENT_LIST_DIR}/../../components/touch/ft6x06
  ${CMAKE_CURRENT_LIST_DIR}/../../components/touch/gt911
  ${CMAKE_CURRENT_LIST_DIR}/../../components/touch/tma525b
  ${CMAKE_CURRENT_LIST_DIR}/../../components/uart
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/camera
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/camera/device
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/camera/device/ap1302
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/camera/device/max9286
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/camera/device/mt9m114
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/camera/device/ov5640
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/camera/device/ov7670
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/camera/device/ov7725
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/camera/device/sccb
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/camera/receiver
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/display
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/display/adv7535
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/display/dbi
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/display/dc
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/display/dc/dbi
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/display/dc/ssd1963
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/display/fbdev
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/display/it6161
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/display/it6263
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/display/sn65dsi83
  ${CMAKE_CURRENT_LIST_DIR}/../../components/video/i2c
  ${CMAKE_CURRENT_LIST_DIR}/../../middleware/fatfs
  ${CMAKE_CURRENT_LIST_DIR}/../../middleware/freemaster
  ${CMAKE_CURRENT_LIST_DIR}/../../middleware/littlefs
  ${CMAKE_CURRENT_LIST_DIR}/../../rtos/freertos/freertos-kernel
  ${CMAKE_CURRENT_LIST_DIR}/cmsis_drivers
  ${CMAKE_CURRENT_LIST_DIR}/drivers
  ${CMAKE_CURRENT_LIST_DIR}/project_template
  ${CMAKE_CURRENT_LIST_DIR}/template
  ${CMAKE_CURRENT_LIST_DIR}/utilities
  ${CMAKE_CURRENT_LIST_DIR}/utilities/debug_console
  ${CMAKE_CURRENT_LIST_DIR}/utilities/debug_console_lite
  ${CMAKE_CURRENT_LIST_DIR}/utilities/format
  ${CMAKE_CURRENT_LIST_DIR}/utilities/incbin
  ${CMAKE_CURRENT_LIST_DIR}/utilities/shell
)

include_if_use(CMSIS_DSP_Include)
include_if_use(CMSIS_DSP_Source)
include_if_use(CMSIS_Device_API_OSTick)
include_if_use(CMSIS_Device_API_RTOS2)
include_if_use(CMSIS_Driver_Include_CAN)
include_if_use(CMSIS_Driver_Include_Ethernet)
include_if_use(CMSIS_Driver_Include_Ethernet_MAC)
include_if_use(CMSIS_Driver_Include_Ethernet_PHY)
include_if_use(CMSIS_Driver_Include_Flash)
include_if_use(CMSIS_Driver_Include_GPIO)
include_if_use(CMSIS_Driver_Include_I2C)
include_if_use(CMSIS_Driver_Include_MCI)
include_if_use(CMSIS_Driver_Include_NAND)
include_if_use(CMSIS_Driver_Include_SAI)
include_if_use(CMSIS_Driver_Include_SPI)
include_if_use(CMSIS_Driver_Include_USART)
include_if_use(CMSIS_Driver_Include_USB_Device)
include_if_use(CMSIS_Driver_Include_USB_Host)
include_if_use(CMSIS_Driver_Include_WiFi)
include_if_use(CMSIS_Include_core_cm)
include_if_use(CMSIS_NN_Source)
include_if_use(CMSIS_RTOS2_RTX)
include_if_use(CMSIS_RTOS2_RTX_LIB)
include_if_use(board_project_template)
include_if_use(component_at_least_one_i2c_mux_device_enabled.MCXA344)
include_if_use(component_button.MCXA344)
include_if_use(component_common_task)
include_if_use(component_ctimer_adapter.MCXA344)
include_if_use(component_ctimer_time_stamp_adapter.MCXA344)
include_if_use(component_enable_pca9544.MCXA344)
include_if_use(component_enable_pca9548.MCXA344)
include_if_use(component_flash_adapter.MCXA344)
include_if_use(component_flash_nor_lpspi.MCXA344)
include_if_use(component_gpio_adapter.MCXA344)
include_if_use(component_i2c_adapter_interface.MCXA344)
include_if_use(component_i2c_mux_pca954x.MCXA344)
include_if_use(component_led.MCXA344)
include_if_use(component_lists.MCXA344)
include_if_use(component_lpi2c_adapter.MCXA344)
include_if_use(component_lpspi_adapter.MCXA344)
include_if_use(component_lptmr_adapter.MCXA344)
include_if_use(component_lptmr_time_stamp_adapter.MCXA344)
include_if_use(component_lpuart_adapter.MCXA344)
include_if_use(component_mcxw_flash_adapter.MCXA344)
include_if_use(component_mem_manager.MCXA344)
include_if_use(component_mem_manager_freertos.MCXA344)
include_if_use(component_mem_manager_legacy.MCXA344)
include_if_use(component_mem_manager_light.MCXA344)
include_if_use(component_messaging.MCXA344)
include_if_use(component_mflash_offchip.MCXA344)
include_if_use(component_mflash_onchip.MCXA344)
include_if_use(component_osa)
include_if_use(component_osa_bm)
include_if_use(component_osa_free_rtos)
include_if_use(component_osa_template_config)
include_if_use(component_ostimer_adapter.MCXA344)
include_if_use(component_ostimer_time_stamp_adapter.MCXA344)
include_if_use(component_panic.MCXA344)
include_if_use(component_reset_adapter.MCXA344)
include_if_use(component_serial_manager.MCXA344)
include_if_use(component_serial_manager_spi.MCXA344)
include_if_use(component_serial_manager_swo.MCXA344)
include_if_use(component_serial_manager_uart.MCXA344)
include_if_use(component_serial_manager_virtual.MCXA344)
include_if_use(component_timer_manager.MCXA344)
include_if_use(device_CMSIS.MCXA344)
include_if_use(device_RTE.MCXA344)
include_if_use(device_project_template.MCXA344)
include_if_use(device_startup.MCXA344)
include_if_use(device_system.MCXA344)
include_if_use(driver_aoi.MCXA344)
include_if_use(driver_cache_lpcac_n4a_mcxn.MCXA344)
include_if_use(driver_camera-common.MCXA344)
include_if_use(driver_camera-device-ap1302.MCXA344)
include_if_use(driver_camera-device-common.MCXA344)
include_if_use(driver_camera-device-max9286.MCXA344)
include_if_use(driver_camera-device-mt9m114.MCXA344)
include_if_use(driver_camera-device-ov5640.MCXA344)
include_if_use(driver_camera-device-ov7670.MCXA344)
include_if_use(driver_camera-device-ov7725.MCXA344)
include_if_use(driver_camera-device-sccb.MCXA344)
include_if_use(driver_camera-receiver-common.MCXA344)
include_if_use(driver_cdog.MCXA344)
include_if_use(driver_clock.MCXA344)
include_if_use(driver_cmsis_gpio.MCXA344)
include_if_use(driver_cmsis_lpi2c.MCXA344)
include_if_use(driver_cmsis_lpspi.MCXA344)
include_if_use(driver_cmsis_lpuart.MCXA344)
include_if_use(driver_common.MCXA344)
include_if_use(driver_crc.MCXA344)
include_if_use(driver_ctimer.MCXA344)
include_if_use(driver_dbi.MCXA344)
include_if_use(driver_dc-fb-common.MCXA344)
include_if_use(driver_dc-fb-dbi.MCXA344)
include_if_use(driver_dc-fb-ssd1963.MCXA344)
include_if_use(driver_display-adv7535.MCXA344)
include_if_use(driver_display-common.MCXA344)
include_if_use(driver_display-it6161.MCXA344)
include_if_use(driver_display-it6263.MCXA344)
include_if_use(driver_display-sn65dsi83.MCXA344)
include_if_use(driver_edma4.MCXA344)
include_if_use(driver_edma_soc.MCXA344)
include_if_use(driver_eim.MCXA344)
include_if_use(driver_ektf2k.MCXA344)
include_if_use(driver_eqdc.MCXA344)
include_if_use(driver_erm.MCXA344)
include_if_use(driver_fbdev.MCXA344)
include_if_use(driver_flexcan.MCXA344)
include_if_use(driver_flexcan_edma.MCXA344)
include_if_use(driver_ft3267.MCXA344)
include_if_use(driver_ft5406.MCXA344)
include_if_use(driver_ft5406_rt.MCXA344)
include_if_use(driver_ft6x06.MCXA344)
include_if_use(driver_fxas21002cq.MCXA344)
include_if_use(driver_fxls8974cf.MCXA344)
include_if_use(driver_fxos8700cq.MCXA344)
include_if_use(driver_glikey.MCXA344)
include_if_use(driver_gpio.MCXA344)
include_if_use(driver_gt911.MCXA344)
include_if_use(driver_htu21d.MCXA344)
include_if_use(driver_icm42688p.MCXA344)
include_if_use(driver_ili9341.MCXA344)
include_if_use(driver_inputmux.MCXA344)
include_if_use(driver_inputmux_connections.MCXA344)
include_if_use(driver_lpadc.MCXA344)
include_if_use(driver_lpc_freqme.MCXA344)
include_if_use(driver_lpcmp.MCXA344)
include_if_use(driver_lpi2c.MCXA344)
include_if_use(driver_lpi2c_edma.MCXA344)
include_if_use(driver_lpi2c_freertos.MCXA344)
include_if_use(driver_lpspi.MCXA344)
include_if_use(driver_lpspi_edma.MCXA344)
include_if_use(driver_lpspi_freertos.MCXA344)
include_if_use(driver_lptmr.MCXA344)
include_if_use(driver_lpuart.MCXA344)
include_if_use(driver_lpuart_edma.MCXA344)
include_if_use(driver_lpuart_freertos.MCXA344)
include_if_use(driver_lsm6dso.MCXA344)
include_if_use(driver_mau.MCXA344)
include_if_use(driver_max30101.MCXA344)
include_if_use(driver_mcx_cmc.MCXA344)
include_if_use(driver_mcx_spc.MCXA344)
include_if_use(driver_mcx_vbat.MCXA344)
include_if_use(driver_mma8451q.MCXA344)
include_if_use(driver_mma8652fc.MCXA344)
include_if_use(driver_nmh1000.MCXA344)
include_if_use(driver_opamp_fast.MCXA344)
include_if_use(driver_ostimer.MCXA344)
include_if_use(driver_p3t1755.MCXA344)
include_if_use(driver_port.MCXA344)
include_if_use(driver_psp27801.MCXA344)
include_if_use(driver_pwm.MCXA344)
include_if_use(driver_reset.MCXA344)
include_if_use(driver_romapi.MCXA344)
include_if_use(driver_rtc.MCXA344)
include_if_use(driver_rtt.MCXA344)
include_if_use(driver_rtt_template.MCXA344)
include_if_use(driver_ssd1963.MCXA344)
include_if_use(driver_st7796s.MCXA344)
include_if_use(driver_tma525b.MCXA344)
include_if_use(driver_trdc_1.MCXA344)
include_if_use(driver_trdc_soc.MCXA344)
include_if_use(driver_tsl2561.MCXA344)
include_if_use(driver_utick.MCXA344)
include_if_use(driver_video-common.MCXA344)
include_if_use(driver_video-i2c.MCXA344)
include_if_use(driver_waketimer.MCXA344)
include_if_use(driver_wuu.MCXA344)
include_if_use(driver_wwdt.MCXA344)
include_if_use(middleware_fatfs)
include_if_use(middleware_fatfs_ram)
include_if_use(middleware_fatfs_template_mmc)
include_if_use(middleware_fatfs_template_nand)
include_if_use(middleware_fatfs_template_ram)
include_if_use(middleware_fatfs_template_sd)
include_if_use(middleware_fatfs_template_sdspi)
include_if_use(middleware_fatfs_template_usb)
include_if_use(middleware_fmstr)
include_if_use(middleware_fmstr_platform_56f800e)
include_if_use(middleware_fmstr_platform_gen32le)
include_if_use(middleware_fmstr_platform_pa32)
include_if_use(middleware_fmstr_platform_s12z)
include_if_use(middleware_fmstr_platform_s32)
include_if_use(middleware_freertos-kernel)
include_if_use(middleware_freertos-kernel_cm33_non_trustzone)
include_if_use(middleware_freertos-kernel_cm33_trustzone_non_secure)
include_if_use(middleware_freertos-kernel_cm33_trustzone_secure)
include_if_use(middleware_freertos-kernel_config)
include_if_use(middleware_freertos-kernel_extension)
include_if_use(middleware_freertos-kernel_heap_1)
include_if_use(middleware_freertos-kernel_heap_2)
include_if_use(middleware_freertos-kernel_heap_3)
include_if_use(middleware_freertos-kernel_heap_4)
include_if_use(middleware_freertos-kernel_heap_5)
include_if_use(middleware_freertos-kernel_mpu_wrappers)
include_if_use(middleware_freertos-kernel_mpu_wrappers_v2)
include_if_use(middleware_freertos-kernel_use_newlib_reentrant)
include_if_use(middleware_freertos-kernel_use_picolibc_tls)
include_if_use(middleware_littlefs)
include_if_use(middleware_littlefs_mflash)
include_if_use(utilities_misc_utilities.MCXA344)
include_if_use(utility_assert.MCXA344)
include_if_use(utility_assert_lite.MCXA344)
include_if_use(utility_debug_console.MCXA344)
include_if_use(utility_debug_console_lite.MCXA344)
include_if_use(utility_debug_console_template_config.MCXA344)
include_if_use(utility_format.MCXA344)
include_if_use(utility_incbin.MCXA344)
include_if_use(utility_shell.MCXA344)
include_if_use(utility_str.MCXA344)
