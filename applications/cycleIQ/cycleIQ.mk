APPSRC += applications/cycleIQ/pas.c \
					applications/cycleIQ/data.c \
					applications/cycleIQ/comm.c \
					applications/cycleIQ/control.c \
					applications/cycleIQ/sensors.c \
					applications/cycleIQ/service.c

APPINC += applications/cycleIQ

include external/cycleiq-protocol/cycleiq_protocol.mk
