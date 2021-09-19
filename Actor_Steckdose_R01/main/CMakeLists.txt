# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS 
	"actor_main.c"
	"wiog_include/wiog_system.c"
	"wiog_include/wiog_data.c"
	"wiog_include/wiog_wifi_actor.c"
	"ns_socket.c"
)
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()
