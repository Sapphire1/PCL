# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Create an executable file from sources:
ADD_LIBRARY(CloudViewer SHARED ${files})
FIND_PACKAGE( PCL 1.7.1 REQUIRED )

# Link external libraries
TARGET_LINK_LIBRARIES(CloudViewer ${DisCODe_LIBRARIES} ${PCL_LIBRARIES})

# Add aother libraries if necessary, for example:
# TARGET_LINK_LIBRARIES(Example ${DisCODe_LIBRARIES} ${OpenCV_LIBS} )

INSTALL_COMPONENT(CloudViewer)
