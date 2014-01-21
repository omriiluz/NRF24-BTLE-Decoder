
SHELL = /bin/sh
CC    = gcc
 
FLAGS        = -std=gnu99
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
