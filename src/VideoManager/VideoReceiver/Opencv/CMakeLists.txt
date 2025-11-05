# In src/VideoManager/VideoReceiver/Opencv/CMakeLists.txt

# 1. Define the name of the target library for this backend

# 2. Add the sources to the target
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    videostreamer.cc
    videostreamer.h
    opencvimageprovider.cc
    opencvimageprovider.h
)


# 4. Include current and parent directory headers
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 5. Conditional Compile Definitions
if(QGC_ENABLE_QT_VIDEOSTREAMING)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE QGC_QT_STREAMING)
endif()

if(NOT QGC_ENABLE_UVC)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE QGC_DISABLE_UVC)
endif()

# 6. Link Libraries (CRITICAL FIX FOR LNK2019)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    # Link to the required OpenCV libraries
    ${OpenCV_LIBS}

    # Required Qt dependencies
    Qt6::Core
    Qt6::Gui
    Qt6::Qml
    Qt6::Quick
)
