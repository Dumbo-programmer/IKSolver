TARGET_LIB = libiksolver.a

ifeq ($(OS),Windows_NT)
EXE_EXT ?= .exe
SDL_CFLAGS ?= -IC:/Users/User/vcpkg/installed/x64-mingw-dynamic/include
SDL_LIBS ?= -LC:/Users/User/vcpkg/installed/x64-mingw-dynamic/lib -lmingw32 -lSDL2main -lSDL2 -mconsole
RM := del /Q /F
OBJ_CLEAN_PATTERN := src\*.o
else
EXE_EXT ?=
SDL_CFLAGS ?= $(shell pkg-config --cflags sdl2 2>/dev/null)
SDL_LIBS ?= $(shell pkg-config --libs sdl2 2>/dev/null)
RM := rm -f
OBJ_CLEAN_PATTERN := $(OBJ_FILES)
endif

TARGET_DEMO = demo$(EXE_EXT)
TARGET_EXAMPLE = basic_demo$(EXE_EXT)
TARGET_TEST = test_solver$(EXE_EXT)

CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -D_USE_MATH_DEFINES
CXXFLAGS += -Iinclude $(SDL_CFLAGS)
LDFLAGS ?= -L.

SRC_DIR = src
EXAMPLES_DIR = examples
TESTS_DIR = tests

SRC_FILES = \
    $(SRC_DIR)/Vector2.cpp \
    $(SRC_DIR)/Vector3.cpp \
    $(SRC_DIR)/IKChain2D.cpp \
    $(SRC_DIR)/IKChain3D.cpp \
    $(SRC_DIR)/CCDSolver.cpp \
    $(SRC_DIR)/FABRIKSolver.cpp \
    $(SRC_DIR)/JacobianSolver.cpp \
    $(SRC_DIR)/Solvers3D.cpp \
    $(SRC_DIR)/JointConstraints.cpp

OBJ_FILES = $(SRC_FILES:.cpp=.o)

all: $(TARGET_LIB) $(TARGET_TEST) $(TARGET_DEMO) $(TARGET_EXAMPLE)

lib: $(TARGET_LIB)

demos: $(TARGET_DEMO) $(TARGET_EXAMPLE)

test: $(TARGET_TEST)

run-tests: $(TARGET_TEST)
	./$(TARGET_TEST)

$(TARGET_LIB): $(OBJ_FILES)
	ar rcs $@ $^

$(TARGET_DEMO): $(SRC_DIR)/maindemo.cpp $(TARGET_LIB)
	$(CXX) $(CXXFLAGS) $< $(LDFLAGS) -liksolver $(SDL_LIBS) -o $@

$(TARGET_EXAMPLE): $(EXAMPLES_DIR)/demo.cpp $(TARGET_LIB)
	$(CXX) $(CXXFLAGS) $< $(LDFLAGS) -liksolver $(SDL_LIBS) -o $@

$(TARGET_TEST): $(TESTS_DIR)/test_solver.cpp $(TARGET_LIB)
	$(CXX) $(CXXFLAGS) $< $(LDFLAGS) -liksolver -o $@

$(SRC_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	-$(RM) $(OBJ_CLEAN_PATTERN) $(TARGET_LIB) $(TARGET_DEMO) $(TARGET_EXAMPLE) $(TARGET_TEST)

.PHONY: all lib demos test run-tests clean
