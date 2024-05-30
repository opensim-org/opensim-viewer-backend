from pygltflib import *
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
    
class ViewVolume:
    def __init__(self, name,) -> None:
        self.name = name
        self.volumeMax = [-100000.0, -100000.0, -100000.0]
        self.volumeMin = [100000.0, 100000.0, 100000.0]

    def updateFromAccessor(self, accessor: Accessor) -> None:
        accessorMin = accessor.min
        accessorMax = accessor.max
        for dim in range(3):
            if accessorMin[dim] < self.volumeMin[dim]: 
                self.volumeMin[dim] = accessorMin[dim]
            if accessorMax[dim] > self.volumeMax[dim]: 
                self.volumeMax[dim] = accessorMax[dim]
            
