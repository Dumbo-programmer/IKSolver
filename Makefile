# -------------------------------
###############################################################
# IKSolver Makefile: Now cross-platform! (Windows, Linux, macOS)
# Because everyone deserves a solver, even penguins and apples.
###############################################################
# -------------------------------

# Targets
TARGET_LIB     = libiksolver.a
TARGET_DEMO    = demo.exe
TARGET_EXAMPLE = basic_demo.exe
TARGET_TEST    = test_solver.exe

ifeq ($(OS),Windows_NT)
    # Windows settings (vcpkg SDL2)
    CXX      = g++
    CXXFLAGS = -std=c++17 -O2 -Iinclude -IC:/Users/User/vcpkg/installed/x64-mingw-dynamic/include -Wall -Wextra -D_USE_MATH_DEFINES
    LDFLAGS  = -L. -LC:/Users/User/vcpkg/installed/x64-mingw-dynamic/lib
    LDLIBS   = -lmingw32 -lSDL2main -lSDL2 -mconsole
    RM       = -del /Q
else
    # Linux/macOS settings (assumes SDL2 installed system-wide)
    CXX      = g++
    CXXFLAGS = -std=c++17 -O2 -Iinclude -Wall -Wextra -D_USE_MATH_DEFINES
    LDFLAGS  = -L.
    LDLIBS   = -lSDL2
    RM       = rm -f
endif

# Directories
SRC_DIR      = src
EXAMPLES_DIR = examples
TESTS_DIR    = tests

# Source files
SRC_FILES = $(SRC_DIR)/Vector2.cpp \
            $(SRC_DIR)/Vector3.cpp \
            $(SRC_DIR)/IKChain2D.cpp \
            $(SRC_DIR)/IKChain3D.cpp \
            $(SRC_DIR)/CCDSolver.cpp \
            $(SRC_DIR)/FABRIKSolver.cpp \
            $(SRC_DIR)/JacobianSolver.cpp \
            $(SRC_DIR)/Solvers3D.cpp \
            $(SRC_DIR)/JointConstraints.cpp

OBJ_FILES = $(SRC_FILES:.cpp=.o)

# -------------------------------
# Default build
# -------------------------------
all: $(TARGET_LIB) $(TARGET_DEMO) $(TARGET_EXAMPLE) $(TARGET_TEST)

# -------------------------------
# Build static library
# -------------------------------
$(TARGET_LIB): $(OBJ_FILES)
	ar rcs $@ $^

# -------------------------------
# Build main demo
# -------------------------------
$(TARGET_DEMO): $(SRC_DIR)/maindemo.cpp $(TARGET_LIB)
	$(CXX) $(CXXFLAGS) $< $(LDFLAGS) -liksolver $(LDLIBS) -o $@

# -------------------------------
# Build example
# -------------------------------
$(TARGET_EXAMPLE): $(EXAMPLES_DIR)/demo.cpp $(TARGET_LIB)
	$(CXX) $(CXXFLAGS) $< $(LDFLAGS) -liksolver $(LDLIBS) -o $@

# -------------------------------
# Build tests
# -------------------------------
$(TARGET_TEST): $(TESTS_DIR)/test_solver.cpp $(TARGET_LIB)
	$(CXX) $(CXXFLAGS) $< $(LDFLAGS) -liksolver -o $@

# -------------------------------
# Object file rule
# -------------------------------
$(SRC_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# -------------------------------
# Clean
# -------------------------------
clean:
	$(RM) $(SRC_DIR)/*.o $(TARGET_LIB) $(TARGET_DEMO) $(TARGET_EXAMPLE) $(TARGET_TEST)
	# Cleaning up: because even solvers need a fresh start.

.PHONY: all clean
###############################################################
# End of Makefile. If you made it this far, reward yourself.
# Or run 'make' and watch the magic happen.
###############################################################
