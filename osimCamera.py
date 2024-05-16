'''
    ---------------------------------------------------------------------------

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

'''
osimCamera class is a utility that embodies the creation and manipulation of a Camera
'''
class osimCamera:
    def __init__(self, name, position=[0., 1.0, 2.0], rotation=[0., 0., 0., 1.], yfov=1.0, znear=1.0, zfar=1000.0):
        self.name = name
        self.position = position
        self.rotation = rotation
        self.yfov = yfov
        self.znear = znear 
        self.zfar = zfar

    def __str__(self):
        return self.name
    
