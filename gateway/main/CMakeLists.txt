# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS 
	"gw_main.c"
	"rs232.c"
	"shotlist.c"
	"wiog_include/wiog_system.c"
	"wiog_include/wiog_data.c"
)
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()
