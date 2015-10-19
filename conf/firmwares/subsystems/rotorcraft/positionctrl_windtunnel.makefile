POSC_WT_CFLAGS = -DPOSITIONCTRL_TYPE_H=\"positionctrl_windtunnel.h\"
POSC_WT_SRCS  = $(SRC_FIRMWARE)/positionctrl_windtunnel.c

ap.CFLAGS += $(POSC_WT_CFLAGS)
ap.srcs += $(POSC_WT_SRCS)

nps.CFLAGS += $(POSC_WT_CFLAGS)
nps.srcs += $(POSC_WT_SRCS)
