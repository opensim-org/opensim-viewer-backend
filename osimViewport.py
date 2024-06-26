'''
    ---------------------------------------------------------------------------
    opensim-viewer-backend: osimViewport.py
    ---------------------------------------------------------------------------

    Copyright 2024 Stanford University and the Authors
    
    Author(s): Ayman Habib
    
    Licensed under the Apache License, Version 2.0 (the "License"); you may not
    use this file except in compliance with the License. You may obtain a copy
    of the License at http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
'''

from pathlib import Path
import osimConverters as osimC
import opensim as osim
import osimConverters.openSimData2Gltf
import osimViewerOptions
'''
osimViewort class is a utility that embodies the creation and manipulation of a view
that will be displayed in a jupyter notebook. the parameters passed in will be used to shape/size and otherwise
customize the viewport in a manner similar to how matplotlib allows users to customize plots at a high level.
One benefit of this approach is that the internals of how the window/canvas is created, are all kept internal to
this class and so can be refactored. For now the customizations will go into one of two places:
- Scene customization: lights, floors, ... 
- UI customization: size, labels, ...

As of now this supports a single model, however, there's nothing preventing having multiple opensim models
in the same gltf file.
'''
class osimViewport:
    def __init__(self, width, hight):
        self.width = width
        self.height = hight
        self._label = ""
        self._options = osimViewerOptions.osimViewerOptions()

    def addModelFile(self, modelFile, options=None):
        self._modelFile = modelFile
        self._motions = []
        self._label = ""
        if options is not None:
            self._options = options

    def addDataFile(self, dataFile, options=None):
        self._modelFile = dataFile #this should be fixed as motion rather than model
        self._motions = []
        self._label = ""
        if options is not None:
            self._options = options

    def addModelAndMotionFiles(self, modelFile, motions, options=None):
        self._motions = motions
        self._modelFile = modelFile
        self._label = ""
        if options is not None:
            self._options = options

    def addSceneCamera(self, sceneCamera):
        self._options.addCamera(sceneCamera)

    def  show(self):
        if (len(self._motions)==0):
            gltfOutput = osimC.convertNativeFileToGLTF(self._modelFile, self._options)
        else:
            gltfOutput = osimC.convertNativeFileSetToGLTF(self._modelFile, self._motions, self._options)
        
        # add user provided cameras 
        for cam in self._options._additionalCameras:
            osimConverters.openSimData2Gltf.addCamera(gltfOutput, cam.name, None, cam.position, cam.rotation)
        
        shortname = Path(self._modelFile).stem
        gltfOutput.save(shortname+'.gltf')
    
