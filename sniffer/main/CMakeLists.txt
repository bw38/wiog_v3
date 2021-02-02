# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS 
	"sniffer_main.c"
	"ln_iog.c"
)
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()
