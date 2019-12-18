# Compile Info
CC := g++
SRCEXT := cpp
CFLAGS := -O2 -std=c++17 -pthread -g -Wall
INCDIR := -I include
LINKS= -lglut -lGL -lGLU

# Source Info, target = cpplox, entry should be in cpplox.cpp
SRCDIR := src
BINDIR := bin
TARGET := main
SOURCES := $(shell find $(SRCDIR) -type f -iname "*.$(SRCEXT)" ! -iname "$(TARGET).$(SRCEXT)")
OBJECTS := $(patsubst $(SRCDIR)/%,$(BINDIR)/%,$(SOURCES:.$(SRCEXT)=.o))

# Compile main
$(TARGET): $(OBJECTS) $(BINDIR)/$(TARGET).o
	@echo "Linking..."
	@echo "$(CC) $^ $(CFLAGS) -o $(TARGET) $(LINKS)"; $(CC) $^ $(CFLAGS) -o $(TARGET) $(LINKS)

$(BINDIR)/%.o: $(SRCDIR)/%.$(SRCEXT) 
	@mkdir -p $(BINDIR)
	@echo "$(CC) $(CFLAGS) $(INCDIR) -c -o $@ $<"; $(CC) $(CFLAGS) $(INCDIR) -c -o $@ $<

# Clean all binary files
clean:
	@echo " Cleaning..."; 
	@echo "$(RM) -r $(BINDIR) $(TARGET)"; $(RM) -r $(BINDIR) $(TARGET)
	@echo "$(RM) -r $(TESTBINDIR)"; $(RM) -r $(TESTBINDIR)

# Declare clean as utility, not a file
.PHONY: clean exec