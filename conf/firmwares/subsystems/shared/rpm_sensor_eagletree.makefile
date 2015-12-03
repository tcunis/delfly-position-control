

RPM_SENSOR_EAGLETREE_SRCS   =  $(SRC_SUBSYSTEMS)/sensors/rpm_sensor.c
RPM_SENSOR_EAGLETREE_SRCS   += $(SRC_ARCH)/subsystems/sensors/rpm_sensor_arch.c

$(TARGET).srcs += $(RPM_SENSOR_EAGLETREE_SRCS)
