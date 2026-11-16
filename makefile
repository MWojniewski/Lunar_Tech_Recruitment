### Makefile

CC = gcc
CFLAGS = -Wall -Wextra -Werror -std=c11 -O2 -Iinclude
LDFLAGS = -lm
PYTHON ?= python

SRC = src/main.c src/physics.c src/pilot.c src/telemetry.c
OBJ = $(SRC:.c=.o)
TARGET = lunar_lander

.PHONY: all clean run plot

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

src/%.o: src/%.c include/lander.h
	$(CC) $(CFLAGS) -c $< -o $@

run: $(TARGET)
	./$(TARGET) results/baseline.csv

plot: scripts/plot_results.py results/baseline.csv
	$(PYTHON) scripts/plot_results.py results/baseline.csv

clean:
	rm -f $(OBJ) $(TARGET)