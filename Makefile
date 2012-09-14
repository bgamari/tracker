#!/usr/bin/make

########################################################################

# *** Fill out this information *** 

#  Project Name
PROJECT = tracker
#  List of the objects files to be compiled/assembled
OBJECTS = main.o dac.o beagle_spi.o feedback.o

# Selected architecture/board
ARCH = stm32/udaq
#ARCH = atsam3s/openmodule

PROJECT_LIBS = -lm

# Programming language for this project (C for C, CPP for C++)
LANGUAGE = C

# Compiling options
DEBUG = -g3
OPTIMIZATION = 0
CFLAGS = -std=c99
LDFLAGS = -lm

########################################################################

# Path to the compiler and library
LIB_BASE := ../libmanyuc/src

# Include libmanyuc's Makefile to take care of the rest
include $(LIB_BASE)/Makefile

