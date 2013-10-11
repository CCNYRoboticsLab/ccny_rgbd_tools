all: rgbdtools_local

GIT_DIR = rgbdtools_git
BUILD_DIR = $(GIT_DIR)/build
TOP_DIR = $(PWD)

GIT_URL =  https://github.com/ccny-ros-pkg/rgbdtools.git
#GIT_REVISION = master

include $(shell rospack find mk)/git_checkout.mk

rgbdtools_local: $(GIT_DIR) 
	mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake ../ -DCMAKE_INSTALL_PREFIX=$(TOP_DIR)
	cd $(BUILD_DIR) && make install 
	touch rgbdtools

clean:
	cd $(BUILD_DIR) && make clean 
	rm -rf rgbdtools rospack_nosubdirs

wipe: clean
	-rm -rf $(GIT_DIR) include lib bin rospack_nosubdirs
