
SHELL = /bin/sh
UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S),Darwin)
CC		= g++
else
CC    	= gcc
FLAGS 	= -std=gnu99
endif

CFLAGS       = -Wall -O3
OUTDIR		 = ./bin
 
TARGET  = $(OUTDIR)/nrf24-btle-decoder
SOURCES = nrf24-btle-decoder.c
 
all: $(TARGET)
	
$(TARGET): $(SOURCES)
	$(CC) $(FLAGS) $(CFLAGS) -o $(TARGET) $(SOURCES)

clean:
	-rm -f $(TARGET) 
	-rm -f $(TARGET).exe
