# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS 
	"sniffer_main.c"
	"../../wiog_include/wiog_system.c"
)
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()
