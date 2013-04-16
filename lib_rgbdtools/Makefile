all: rgbdtools_local

GIT_DIR   = rgbdtools_git
BUILD_DIR = $(GIT_DIR)/build
TOP_DIR = $(PWD)

GIT_URL =  https://github.com/ccny-ros-pkg/rgbdtools.git

G2O_INCLUDE_DIRS  = $(shell rospack find ccny_g2o)/include
G2O_LIBRARY_DIRS  = $(shell rospack find ccny_g2o)/lib
G2O_LIBRARIES = 'g2o_core;g2o_stuff;g2o_solver_cholmod;g2o_solver_csparse;g2o_types_slam3d;cholmod'

include $(shell rospack find mk)/git_checkout.mk

rgbdtools_local: $(GIT_DIR) 
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake ../ -DCMAKE_INSTALL_PREFIX=$(TOP_DIR) -DG2O_INCLUDE_DIRS=$(G2O_INCLUDE_DIRS) -DG2O_LIBRARY_DIRS=$(G2O_LIBRARY_DIRS) -DG2O_LIBRARIES=$(G2O_LIBRARIES) 
	cd $(BUILD_DIR) && make install 
	touch rgbdtools

clean:
	cd $(BUILD_DIR) && make clean 
	rm -rf rgbdtools rospack_nosubdirs

wipe: clean
	-rm -rf $(GIT_DIR) include lib bin rospack_nosubdirs
