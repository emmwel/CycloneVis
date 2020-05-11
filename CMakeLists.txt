#--------------------------------------------------------------------
# Inviwo cyclonevis Module
ivw_module(cyclonevis)

#--------------------------------------------------------------------
# Add header files
set(HEADER_FILES
    include/inviwo/cyclonevis/cyclonevismodule.h
    include/inviwo/cyclonevis/cyclonevismoduledefine.h
    include/inviwo/cyclonevis/ngraph/ngraph.hpp
    include/inviwo/cyclonevis/ngraph/set_ops.hpp
    include/inviwo/cyclonevis/processors/cyclonecontourlines.h
    include/inviwo/cyclonevis/processors/floodfillvolume.h
    include/inviwo/cyclonevis/processors/meshgraph.h
    include/inviwo/cyclonevis/processors/volumeslicequad.h
    include/inviwo/cyclonevis/util/batchvolumesampler.h
)
ivw_group("Header Files" ${HEADER_FILES})

#--------------------------------------------------------------------
# Add source files
set(SOURCE_FILES
    src/cyclonevismodule.cpp
    src/processors/cyclonecontourlines.cpp
    src/processors/floodfillvolume.cpp
    src/processors/meshgraph.cpp
    src/processors/volumeslicequad.cpp
    src/util/batchvolumesampler.cp
)
ivw_group("Source Files" ${SOURCE_FILES})

#--------------------------------------------------------------------
# Add shaders
set(SHADER_FILES
)
ivw_group("Shader Files" ${SHADER_FILES})

#--------------------------------------------------------------------
# Add Unittests
set(TEST_FILES
    tests/unittests/cyclonevis-unittest-main.cpp
)
ivw_add_unittest(${TEST_FILES})

#--------------------------------------------------------------------
# Create module
ivw_create_module(${SOURCE_FILES} ${HEADER_FILES} ${SHADER_FILES})

#--------------------------------------------------------------------
# Add shader directory to pack
# ivw_add_to_module_pack(${CMAKE_CURRENT_SOURCE_DIR}/glsl)
