idf_component_register(SRCS "robot_arm.c"
                       INCLUDE_DIRS "."
                       REQUIRES tb6600_stepper kinematics_rrr sensor_angulo_magnetico)
