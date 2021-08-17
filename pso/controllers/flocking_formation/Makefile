
CFLAGS = -Wno-comment -Wno-unused-variable
INCLUDE = -I"../gsl/include" -I"../localization"
LIBRARIES = -L"../gsl/lib" -lgsl -lgslcblas -lm

### Do not modify: this includes Webots global Makefile.include
C_SOURCES = ../const.c ../localization/gsl_helper.c kalman_variable.c kalman_vel.c kalman_acc.c odometry.c flockform_controller.c 
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
