#
#  makefile for fe_dump (Linux, macOS, Windows MSYS2 UCRT64)
#

CC = gcc
CFLAGS = -std=c99 -O2 -Wall
LDLIBS = -lusb-1.0
TARGET = fe_dump

ifeq ($(OS),Windows_NT)
    LDLIBS = -static -lusb-1.0
    TARGET = fe_dump.exe
endif

$(TARGET): fe_dump.c
	$(CC) $(CFLAGS) -o $@ $< $(LDLIBS)

clean:
	rm -f fe_dump fe_dump.exe
