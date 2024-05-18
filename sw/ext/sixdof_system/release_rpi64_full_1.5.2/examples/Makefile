# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++14 -I../include

# Paths and libraries
LDFLAGS = -L../rpi64/ -lSdsDroneSdk -Wl,-rpath,../rpi64/

# Source files and targets
# SOURCES = sixdof_tracking.cpp \
#           relative_beacon_tracking.cpp \
#           relative_angle_tracking.cpp \
#           field_of_view_report.cpp \
#           gyro_calibration.cpp \
#           heartbeat.cpp \
#           heartbeat_manager.cpp

# Source files and targets
SOURCES = custom_mode_am.cpp 

OBJECTS = $(SOURCES:.cpp=.o)

TARGETS = custom_mode_am 

# Default target
all: $(TARGETS)

# Pattern rule for object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Rules for each executable
custom_mode_am: custom_mode_am.o
	$(CXX) -o $@ $^ $(LDFLAGS)

# sixdof_tracking: sixdof_tracking.o
# 	$(CXX) -o $@ $^ $(LDFLAGS)

# relative_beacon_tracking: relative_beacon_tracking.o
# 	$(CXX) -o $@ $^ $(LDFLAGS)

# relative_angle_tracking: relative_angle_tracking.o
# 	$(CXX) -o $@ $^ $(LDFLAGS)

# field_of_view_report: field_of_view_report.o
# 	$(CXX) -o $@ $^ $(LDFLAGS)

# gyro_calibration: gyro_calibration.o
# 	$(CXX) -o $@ $^ $(LDFLAGS)

# heartbeat: heartbeat.o
# 	$(CXX) -o $@ $^ $(LDFLAGS)

# heartbeat_manager: heartbeat_manager.o
# 	$(CXX) -o $@ $^ $(LDFLAGS)

# Clean up
clean:
	rm -f $(OBJECTS) $(TARGETS)
