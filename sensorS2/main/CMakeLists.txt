# Set usual component variables
set(COMPONENT_SRCS 		
		"sensor_main.c" 
		"wiog_include/wiog_system.c"
    	"wiog_include/wiog_data.c"
    	"wiog_include/wiog_wifi_sensor.c"
    	"gadget/ubat/ubat.c"
)
    	
set(COMPONENT_ADD_INCLUDEDIRS "")

set(COMPONENT_REQUIRES 
		soc 
		nvs_flash 
		ulp driver 
		esp_adc_cal
)

register_component()

#
# ULP support additions to component CMakeLists.txt.
#
# 1. The ULP app name must be unique (if multiple components use ULP).
set(ulp_app_name ulp_${COMPONENT_NAME})
#
# 2. Specify all C and Assembly source files.
#    Files should be placed into a separate directory (in this case, ulp/),
#    which should not be added to COMPONENT_SRCS.
set(ulp_riscv_S_sources 
		"ulp/riscv_sht31/riscv_sht31.c"  
)

#
# 3. List all the component source files which include automatically
#    generated ULP export file, ${ulp_app_name}.h:
set(ulp_exp_dep_srcs 
	"sensor_main.c"
)

#
# 4. Call function to build ULP binary and embed in project using the argument
#    values above.
ulp_embed_binary(${ulp_app_name} "${ulp_riscv_S_sources}" "${ulp_exp_dep_srcs}")
