TARGET_LIB = libiksolver.a
.RECIPEPREFIX := >

ifeq ($(OS),Windows_NT)
EXE_EXT ?= .exe
SDL_CFLAGS ?= -IC:/Users/User/vcpkg/installed/x64-mingw-dynamic/include
SDL_LIBS ?= -LC:/Users/User/vcpkg/installed/x64-mingw-dynamic/lib -lmingw32 -lSDL2main -lSDL2 -mconsole
RM_FILES := del /Q /F
MKDIR_OBJ = if not exist "$(OBJ_DIR)" mkdir "$(OBJ_DIR)"
RMDIR_OBJ = if exist "$(OBJ_DIR)" rmdir /S /Q "$(OBJ_DIR)"
LEGACY_OBJ_GLOB := src\*.o
else
EXE_EXT ?=
SDL_CFLAGS ?= $(shell pkg-config --cflags sdl2 2>/dev/null)
SDL_LIBS ?= $(shell pkg-config --libs sdl2 2>/dev/null)
RM_FILES := rm -f
MKDIR_OBJ = mkdir -p $(OBJ_DIR)
RMDIR_OBJ = rm -rf $(OBJ_DIR)
LEGACY_OBJ_GLOB := src/*.o
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
OBJ_DIR = build/obj

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

OBJ_FILES = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRC_FILES))

all: $(TARGET_LIB) $(TARGET_TEST) $(TARGET_DEMO) $(TARGET_EXAMPLE)

lib: $(TARGET_LIB)

demos: $(TARGET_DEMO) $(TARGET_EXAMPLE)

test: $(TARGET_TEST)

run-tests: $(TARGET_TEST)
>./$(TARGET_TEST)

$(TARGET_LIB): $(OBJ_FILES)
>ar rcs $@ $^

$(TARGET_DEMO): $(SRC_DIR)/maindemo.cpp $(TARGET_LIB)
>$(CXX) $(CXXFLAGS) $< $(LDFLAGS) -liksolver $(SDL_LIBS) -o $@

$(TARGET_EXAMPLE): $(EXAMPLES_DIR)/demo.cpp $(TARGET_LIB)
>$(CXX) $(CXXFLAGS) $< $(LDFLAGS) -liksolver $(SDL_LIBS) -o $@

$(TARGET_TEST): $(TESTS_DIR)/test_solver.cpp $(TARGET_LIB)
>$(CXX) $(CXXFLAGS) $< $(LDFLAGS) -liksolver -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
>@$(MKDIR_OBJ)
>$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
>-$(RM_FILES) $(TARGET_LIB) $(TARGET_DEMO) $(TARGET_EXAMPLE) $(TARGET_TEST)
>-$(RM_FILES) $(LEGACY_OBJ_GLOB)
>-$(RMDIR_OBJ)

.PHONY: all lib demos test run-tests clean
