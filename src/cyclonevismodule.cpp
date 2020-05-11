/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2020 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include <inviwo/cyclonevis/cyclonevismodule.h>
#include <inviwo/cyclonevis/processors/cyclonecontourlines.h>
#include <inviwo/cyclonevis/processors/floodfillvolume.h>
#include <inviwo/cyclonevis/processors/meshgraph.h>
#include <inviwo/cyclonevis/processors/volumeslicequad.h>

namespace inviwo {

cyclonevisModule::cyclonevisModule(InviwoApplication* app) : InviwoModule(app, "cyclonevis") {
    registerProcessor<FloodFillVolume>();
    registerProcessor<CycloneContourLines>();
    registerProcessor<MeshGraph>();
    // Add a directory to the search path of the Shadermanager
    // ShaderManager::getPtr()->addShaderSearchPath(getPath(ModulePath::GLSL));

    // Register objects that can be shared with the rest of inviwo here:

    // Processors
    registerProcessor<VolumeSliceQuad>();
    // registerProcessor<cyclonevisProcessor>();

    // Properties
    // registerProperty<cyclonevisProperty>();

    // Readers and writes
    // registerDataReader(std::make_unique<cyclonevisReader>());
    // registerDataWriter(std::make_unique<cyclonevisWriter>());

    // Data converters
    // registerRepresentationConverter(std::make_unique<cyclonevisDisk2RAMConverter>());

    // Ports
    // registerPort<cyclonevisOutport>();
    // registerPort<cyclonevisInport>();

    // PropertyWidgets
    // registerPropertyWidget<cyclonevisPropertyWidget, cyclonevisProperty>("Default");

    // Dialogs
    // registerDialog<cyclonevisDialog>(cyclonevisOutport);

    // Other things
    // registerCapabilities(std::make_unique<cyclonevisCapabilities>());
    // registerSettings(std::make_unique<cyclonevisSettings>());
    // registerMetaData(std::make_unique<cyclonevisMetaData>());
    // registerPortInspector("cyclonevisOutport", "path/workspace.inv");
    // registerProcessorWidget(std::string processorClassName, std::unique_ptr<ProcessorWidget> processorWidget); 
    // registerDrawer(util::make_unique_ptr<cyclonevisDrawer>());
}

}  // namespace inviwo
