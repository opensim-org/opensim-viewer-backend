'''
    ---------------------------------------------------------------------------
    opensim-viewer-backend: osimViewerOptions.py
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

'''
osimViewerOptions is a container for a set of knobs used to customize the creation of 
the gltf files from OpenSim native files. Examples of customization would include
- Colors and material props for meshes
- Additional lights
- Additional Cameras
- CameraTrajectories
... this list will grow as demand arises ...

An instance of this class is passed down to the low level functions/utilities that create the gltf file
to bake user options into the file.

Ideally the options made available to users are those supported by the gltf format directly so
we're not reinventing the wheel.
gltf file specification is here 
 ...
'''
class osimViewerOptions:
    def __init__(self):
        self._additionalLights = []
        self._additionalCameras = []
        self._experimentalMarkerShape = 'sphere'
        self._experimentalMarkerColor = ""
        self._forceShape = 'arrow'

    def setExperimentalMarkerShape(self, markerShapeString):
        self._experimentalMarkerShape = markerShapeString

    def getExperimentalMarkerShape(self):
        return self._experimentalMarkerShape

    def  setExperimentalMarkerColor(self, rgb):
        self._experimentalMarkerColor = rgb

    def getExperimentalMarkerColor(self):
        return self._experimentalMarkerColor
    
    def setForceShape(self, shapeString):
        self._forceShape = shapeString 

    def getForceShape(self): 
        return self._forceShape

    def addCamera(self, camera):
        self._additionalCameras.append(camera)
