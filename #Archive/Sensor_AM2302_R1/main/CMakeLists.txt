# ------------------------------------------------------------- 
# Beinhaltet alle Module  der Sensor-Farm
#
# Anpassen an die zu verwendenden Module !!!!!
# durch auskommentieren der nicht verwendeten Sensor-Module
#
# -------------------------------------------------------------


idf_component_register(
	SRCS 
		"sensor_main.c" 
    	"wiog_include/wiog_system.c"
    	"wiog_include/wiog_data.c"
    	"wiog_include/wiog_wifi_sensor.c"
#    	
		"gadget/ubat/ubat.c" 		
#		"gadget/ds18b20/ds18b20.c"
		"gadget/am2302/am2302.c"	
		
	INCLUDE_DIRS 
		""
#	REQUIRES soc nvs_flash ulp
)

# ULP support additions to component CMakeLists.txt.
set(ulp_app_name ulp_${COMPONENT_NAME})

# .S - Files im ulp-Folder unterhalb von main
# nur eine ULP-Auswahl möglich 
set(ulp_s_sources 
#		"ulp/ds18b20.S"
		"ulp/am2302_serialbus.S"
#		"button/ulp/button.S"
#		"stepup/ulp/stepup.S"
#		"ne555_level/ulp/ds18b20.S"
)

# Module, mit Zugriff auf ULP-Variablen
set(ulp_exp_dep_srcs 
		"sensor_main.c"
		"am2302/am2302.c"
#		"ds18b20.c"
#		"button.c"
#		"ne555_level/ne555.c"
)

ulp_embed_binary(${ulp_app_name} ${ulp_s_sources} ${ulp_exp_dep_srcs})
