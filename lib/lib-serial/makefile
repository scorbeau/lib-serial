################################################################################
# Author: Sebastien CORBEAU (corbeau.sebastien@yahoo.fr)                       #
# Date: 19/12/2023                                                             #
################################################################################

################################################################################
# Define project directories
################################################################################
PROJECT_DIR		= $(abspath ..)
RESOURCE_DIR	= $(PROJECT_DIR)/resources
SRC_DIR			= $(abspath ./srcs)

################################################################################
# Define target
################################################################################
ifdef TARGET
	-include $(RESOURCE_DIR)/toolchain/config_$(TARGET).mk
endif

CC		= $(CROSS)gcc
LD		= $(CROSS)ld
AR		= $(CROSS)ar
STRIP	= $(CROSS)strip
ARCH	= $(shell $(CC) -dumpmachine | cut -d - -f1)

################################################################################
# Select compilation configuration
################################################################################
# if DEBUG_LEVEL is passed in make command force enabling
ifneq ($(DEBUG_LEVEL),)
	DEBUG_ENABLE=y
endif

DEBUG_LEVEL	?= 3

ifeq ($(DEBUG_ENABLE),y)
	CFLAGS		+= -g$(DEBUG_LEVEL) -DCONF=debug -DDEBUG_ENABLE=1
	CONF=debug
else
	CONF=release
endif

################################################################################
# Define build directories
################################################################################
BUILD_DIR	?= $(PROJECT_DIR)/build/$(MODE)/$(ARCH)
OBJ_DIR		?= $(BUILD_DIR)/obj
LIB_DIR		?= $(BUILD_DIR)/lib
BIN_DIR		?= $(BUILD_DIR)/bin
TEST_DIR	?= $(BUILD_DIR)/test
DOC_DIR		?= $(PROJ_DIR)/build/doc

################################################################################
# Compilation flags
################################################################################
CFLAGS		+= -funwind-tables -fstack-protector-all -Wall -Werror
INCFLAGS	= -iquote $(abspath ./incs)

################################################################################
# Obj to compile
################################################################################
OBJS_LIST	= $(patsubst %.c, $(OBJ_DIR)/%.o, $(shell find $(SRC_DIR) -name \
				*.c | sed -e 's,\$(SRC_DIR)/,,'))

################################################################################
# Library name
################################################################################
LIB_NAME	?= $(notdir $(shell git rev-parse --show-toplevel))
LIB_BIN		= $(LIB_NAME).a

################################################################################
# Main rules
################################################################################
default: all

all: lib 

################################################################################
# Build rules
################################################################################
lib: $(LIB_BIN)
$(LIB_BIN): $(LIB_DIR)/$(LIB_BIN)
$(LIB_DIR)/%.a: $(OBJS_LIST)
	mkdir -p $(@D)
	$(AR) rcs $@ $^

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	mkdir -p  $(shell dirname $@)
	$(CC) $(CFLAGS) -fPIC $(INCFLAGS) -o $@ -c $<

################################################################################
# Clean rules
################################################################################
clean:
	rm -rf $(LIB_DIR)/$(LIB_BIN) $

distclean: clean
	rm -Rf $(OBJ_DIR)

.PHONY: default all lib clean distclean doc
