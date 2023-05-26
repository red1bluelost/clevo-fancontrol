vpath %.c ../src

CPP = g++
CPPFLAGS = -c -Wall -Wextra -Wpedantic -Wconversion -Wshadow -std=c++20 -O3
LDFLAGS =

DSTDIR := /usr/local
OBJDIR := obj
SRCDIR := src

SRC = clevo_fan_control.cpp
OBJ = $(patsubst %.cpp,$(OBJDIR)/%.o,$(SRC))

TARGET = bin/clevo_fan_control

all: $(TARGET) $(TARGETCPP)

install: $(TARGET)
	@echo Install to ${DSTDIR}/bin/
	@sudo install -m 4750 -g adm $(TARGET) ${DSTDIR}/bin/

test: $(TARGET)
	@sudo chown root $(TARGET)
	@sudo chgrp adm  $(TARGET)
	@sudo chmod 4750 $(TARGET)

$(TARGET): $(OBJ) Makefile
	@mkdir -p bin
	@echo linking $(TARGET) from $(OBJ)
	@$(CPP) $(OBJ) -o $(TARGET) $(LDFLAGS) -lm -lfmt

clean:
	rm $(OBJ) $(TARGET)

$(OBJDIR)/%.o : $(SRCDIR)/%.cpp Makefile
	@echo compiling $<
	@mkdir -p obj
	@$(CPP) $(CPPFLAGS) -c $< -o $@

#$(OBJECTS): | obj

#obj:
#	@mkdir -p $@
