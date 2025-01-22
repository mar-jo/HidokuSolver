CXX = g++
CXXFLAGS = -std=c++20 -Wall -Wextra -O3 
LDFLAGS = -lstdc++fs # Filesystem library for GCC

SRCDIR = .
OBJDIR = obj
BINDIR = bin
TESTDIR = puzzles
SOURCES = $(wildcard $(SRCDIR)/*.cpp) 
OBJECTS = $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR)/%.o, $(SOURCES))
TARGET = $(BINDIR)/hidoku

all: $(TARGET)

$(TARGET): $(OBJECTS)
	@mkdir -p $(BINDIR)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp
	@mkdir -p $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

test: $(TARGET)
	@echo "Running all puzzles in $(TESTDIR)..."
	@for file in $(wildcard $(TESTDIR)/*.txt); do \
		echo "Testing $$file..."; \
		./$(TARGET) $$file || exit 1; \
	done
	@echo "All tests completed successfully!"

run: $(TARGET)
	@if [ -z "$(FILE)" ]; then \
		echo "Please specify a FILE to run, e.g., make run FILE=filename.txt"; \
		exit 1; \
	fi
	@echo "Running $(FILE)..."
	./$(TARGET) $(TESTDIR)/$(FILE)

clean:
	rm -rf $(OBJDIR) $(BINDIR)

.PHONY: all clean test run
