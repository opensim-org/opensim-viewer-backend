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