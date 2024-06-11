import opensim as osim
from pygltflib import *
import numpy as np
import vtk
from .openSimData2Gltf import *

# Class to convert osim model file to a GLTF structure.
# The typical workflow would be to instantiate this object, then traverse the model
# adding artifacts to gltf structure cached in this class
# for all geometry objects, wil create a node for the mesh as a child of the frame it lives on
# the node for the frame will correspond to a bodyId, and relative transform
class DecorativeGeometryImplementationGltf(osim.simbody.DecorativeGeometryImplementation) :
    unitConversion = 1.0    #1.0 if model is in meters, else set to conversionToMeters factor
    gltf = None             # resulting  GLTF object used to accumulate nodes, meshes, cameras etc.
    currentComponent = None # Keep track of which OpenSim::Component being processed so correct annotation is associated
    mapMobilizedBodyIndexToNodes = {}
    mapMobilizedBodyIndexToNodeIndex = {}
    processingPath = False
    currentPathMaterial = None
    mapPathToMaterialIndex = {}
    mapPathsToNodeIds = {}
    useTRS = False  # indicate whether transforms should be written as trs or as a matrix

    modelNodeIndex = None   # index for root node of the model
    modelNode = None        # reference to the root node representing the model
    groundNode = None       # Node corresponding to Model::Ground
    modelState = None       # reference to state object obtained by initSystem
    mapTypesToMaterialIndex = {} # Some opensimTypes share material e.g. Markers
    MaterialGrouping = {}
    model = None

    accessors = None        # references to arrays within the gltf structure for convenience
    buffers = None
    bufferViews = None
    nodes = None
    meshes = None
    animations = None

    def setUnitConversion(self, unitConversion):
        self.unitConversion = unitConversion

    def setGltf(self, gltf):
        self.gltf = gltf
        self.accessors = self.gltf.accessors
        self.buffers = self.gltf.buffers
        self.bufferViews = self.gltf.bufferViews
        self.meshes = self.gltf.meshes
        self.nodes = self.gltf.nodes
        self.materials = self.gltf.materials
        self.animations = self.gltf.animations
    
    def setCurrentComponent(self, component):
        """Keep track of current OpenSim Component being processed, generally the code
        is agnostic to what component it's handling except for the case where multiple calls 
        are needed to render one component as is the case with a muscle/path """
        self.currentComponent = component

    def setDecorativeGeometryIndex(self, index):
        self.dg_index = index

    def setState(self, modelState):
        self.modelState = modelState

    def setDisplayHints(self, displayHints):
        self.displayHints = displayHints

    def get_GLTF(self):
        return self.gltf;

    def implementPointGeometry(self, arg0):
        return _simbody.DecorativeGeometryImplementation_implementPointGeometry(self, arg0)

    def implementLineGeometry(self, arg0):
        # point1 = arg0.getPoint1()
        # point2 = arg0.getPoint2()
        # mesh = self.createGLTFLineStrip(point1, point2)
        # self.meshes.append(mesh)
        # meshId = len(self.meshes)-1
        # meshNode = Node(name=self.currentComponent.getAbsolutePathString()+str(meshId))
        # meshNode.mesh = meshId;
        # nodeIndex = len(self.nodes)
        # self.createExtraAnnotations(meshNode)
        # self.nodes.append(meshNode)
        # self.modelNode.children.append(nodeIndex)
        self.createNodeForLineSegment(arg0)
        return 

    def implementBrickGeometry(self, arg0):
        """Create GLTF artifacts for a brick that includes 
        - node that refers to underlying mesh, and transform that goes with it
        """
        brickData = vtk.vtkCubeSource()
        lengths = arg0.getHalfLengths();
        brickData.SetXLength(lengths.get(0)*2*self.unitConversion)
        brickData.SetYLength(lengths.get(1)*2*self.unitConversion)
        brickData.SetZLength(lengths.get(2)*2*self.unitConversion)
        brickData.Update()
        polyDataOutput = brickData.GetOutput();
        self.createGLTFNodeAndMeshFromPolyData(arg0, "Brick:", polyDataOutput, self.getMaterialIndexByType())
        # print("produce brick")
        return 
    
    def implementCylinderGeometry(self, arg0):
        cylData = vtk.vtkCylinderSource()
        cylData.SetRadius(arg0.getRadius()*self.unitConversion)
        cylData.SetHeight(arg0.getHalfHeight()*self.unitConversion)
        cylData.Update()
        polyDataOutput = cylData.GetOutput();
        self.createGLTFNodeAndMeshFromPolyData(arg0, "Cylinder:", polyDataOutput, self.getMaterialIndexByType())
        # print("produce cylinder", arg0.getHalfHeight(), arg0.getRadius())
        return

    def implementCircleGeometry(self, arg0):
        return _simbody.DecorativeGeometryImplementation_implementCircleGeometry(self, arg0)

    def implementSphereGeometry(self, arg0):
        sphereSource = vtk.vtkSphereSource()
        sphereSource.SetRadius(arg0.getRadius()*self.unitConversion)
        sphereSource.SetPhiResolution(16)
        sphereSource.SetThetaResolution(16)
        sphereSource.Update()
        polyDataOutput = sphereSource.GetOutput()
        self.createGLTFNodeAndMeshFromPolyData(arg0, "Sphere:", polyDataOutput, self.getMaterialIndexByType())

    def implementEllipsoidGeometry(self, arg0):
        sphereSource = vtk.vtkSphereSource()
        sphereSource.SetRadius(1.0*self.unitConversion)
        sphereSource.SetPhiResolution(16)
        sphereSource.SetThetaResolution(16)
        # Make a stretching transform to take the sphere into an ellipsoid
        stretch = vtk.vtkTransformPolyDataFilter();
        stretchSphereToEllipsoid = vtk.vtkTransform();
        radiiVec3 = arg0.getRadii()
        stretchSphereToEllipsoid.Scale(radiiVec3[0], radiiVec3[1], radiiVec3[2]);
        stretch.SetTransform(stretchSphereToEllipsoid);
        stretch.SetInputConnection(sphereSource.GetOutputPort());
        stretch.Update()
        polyDataOutput = stretch.GetOutput()
        self.createGLTFNodeAndMeshFromPolyData(arg0, "Ellipsoid:", polyDataOutput, self.getMaterialIndexByType())
        return

    def implementFrameGeometry(self, arg0):
        # print("produce frame", arg0.getAxisLength())
        return

    def implementTextGeometry(self, arg0):
        return _simbody.DecorativeGeometryImplementation_implementTextGeometry(self, arg0)

    def implementMeshGeometry(self, arg0):
        return _simbody.DecorativeGeometryImplementation_implementMeshGeometry(self, arg0)

    def implementMeshFileGeometry(self, arg0):
        """Function to generate the gltf artifacts corresponding to the passed in mesh file
        This could be expanded to support all formats readable by vtk though using
        and linking vtk for this purpose is a bit of an overkill.

        Args:
            arg0 (DecorativeMeshFile): full path of a file containing mesh

        Raises:
            ValueError: _description_
        """
        if (arg0.getMeshFile().casefold().endswith(".vtp")):
            reader = vtk.vtkXMLPolyDataReader()
            reader.SetFileName(arg0.getMeshFile())
            reader.Update()
            polyDataOutput = reader.GetOutput()
        elif (arg0.getMeshFile().casefold().endswith(".stl")):
            reader = vtk.vtkSTLReader()
            reader.SetFileName(arg0.getMeshFile())
            reader.Update()
            polyDataOutput = reader.GetOutput()
        elif (arg0.getMeshFile().casefold().endswith(".obj")):
            reader = vtk.vtkOBJReader()
            reader.SetFileName(arg0.getMeshFile())
            reader.Update()
            polyDataOutput = reader.GetOutput()
        else:
            raise ValueError("Unsupported file extension")
        self.createGLTFNodeAndMeshFromPolyData(arg0, "Mesh"+arg0.getMeshFile(), polyDataOutput, self.getMaterialIndexByType())
            #     InlineData, SaveNormal, SaveBatchId);
            # rendererNode["children"].emplace_back(nodes.size() - 1);
            # size_t oldTextureCount = textures.size();
            # WriteTexture(buffers, bufferViews, textures, samplers, images, pd, aPart,
            # this->FileName, this->InlineData, textureMap);
            # meshes[meshes.size() - 1]["primitives"][0]["material"] = materials.size();
            # WriteMaterial(materials, oldTextureCount, oldTextureCount != textures.size(), aPart);
        
        return
    
    def getNodeIndexForBody(self, body):
        """Retrieve the index of a gltf-node corresponding to a MobilizedBody

        Args:
            body (OpenSim::Body): _description_

        Returns:
            int: index in gltf[nodes]
        """
        return self.mapMobilizedBodyIndexToNodeIndex[body.getMobilizedBodyIndex()]
    
    def createGLTFNodeAndMeshFromPolyData(self, arg0, gltfName, polyDataOutput, materialIndex)->int:
        if (polyDataOutput.GetNumberOfCells() > 0):
            mesh = self.addMeshForPolyData(polyDataOutput, materialIndex) # populate from polyDataOutput
            self.meshes.append(mesh)
            meshId = len(self.meshes)-1
            meshNode = Node(name=gltfName)
            if (self.useTRS):
                t, r, s = self.createTRSFromTransform(arg0.getTransform(), arg0.getScaleFactors())
                meshNode.translation = t
                meshNode.scale = s
                meshNode.rotation = r
            else:
                meshNode.matrix = self.createMatrixFromTransform(arg0.getTransform(), arg0.getScaleFactors())

            meshNode.mesh = meshId;
            nodeIndex = len(self.nodes)
            self.createExtraAnnotations(meshNode)
            self.nodes.append(meshNode)
            self.mapMobilizedBodyIndexToNodes[arg0.getBodyId()].children.append(nodeIndex)
            return nodeIndex
        return -1
    

    def createNodeForLineSegment(self, arg0):
        """_summary_

        Args:
            arg0 (_type_): DecorativeLine
            gltfName (_type_): _description_

        Returns:
            int: index of gltf-node corresponding to passed in line 
        """
        point1 = arg0.getPoint1()
        point2 = arg0.getPoint2()
        mesh = self.createGLTFLineStrip(osim.Vec3(0.), osim.Vec3(0.0, 1., 0.))
        self.meshes.append(mesh)
        meshId = len(self.meshes)-1
        # Now compute the transform as TRS
        # Now will use point1 and point2 to compute TRS to take unit line in x direction to point1-point2 in space
        newY = point2.to_numpy() - point1.to_numpy()
        newYNorm= np.linalg.norm(newY)
        newYNormalized = newY / newYNorm
        origYNormalized = [0.0, 1.0, 0.]
        newZ = np.cross(origYNormalized, newYNormalized)
        newZNormalized = newZ / np.linalg.norm(newZ)
        newXNormalized = np.cross(newYNormalized, newZNormalized)
        rot33 = osim.Mat33()
        for row in range(3):
            rot33.set(row, 0, newXNormalized[row])
            rot33.set(row, 1, newYNormalized[row])
            rot33.set(row, 2, newZNormalized[row])

        rot = osim.Rotation()
        rot.setRotationFromApproximateMat33(rot33)
        qs = rot.convertRotationToQuaternion()
        rotation = [qs.get(1), qs.get(2), qs.get(3), qs.get(0)]
        # translation is point1
        t = [point1.get(0), point1.get(1), point1.get(2)]
        # scale is vec2Norm
        s = newYNorm
        nodeIndex = len(self.nodes)
        pathSegmentNode = Node(name="pathsegment:")
        pathSegmentNode.scale = [1.0, s, 1.0]
        pathSegmentNode.rotation = rotation
        pathSegmentNode.translation = t
        pathSegmentNode.mesh = meshId
        self.nodes.append(pathSegmentNode)
        self.modelNode.children.append(nodeIndex)



    def implementTorusGeometry(self, arg0):
        torus=vtk.vtkParametricTorus();
        torusSource = vtk.vtkParametricFunctionSource();
        torusSource.SetParametricFunction(torus);
        torusMapper=vtk.vtkPolyDataMapper();
        torusMapper.SetInputConnection(torusSource.GetOutputPort());
        torus.SetRingRadius(arg0.getTorusRadius()+arg0.getTubeRadius());
        torus.SetCrossSectionRadius(arg0.getTubeRadius());
        polyDataOutput = torusSource.GetOutput();
        self.createGLTFNodeAndMeshFromPolyData(arg0, "Torus:", polyDataOutput, self.getMaterialIndexByType())
        return
        
    def implementArrowGeometry(self, arg0):
        return _simbody.DecorativeGeometryImplementation_implementArrowGeometry(self, arg0)

    def implementConeGeometry(self, arg0):
        return _simbody.DecorativeGeometryImplementation_implementConeGeometry(self, arg0)
    
    def addModelNode(self, model):
        self.modelNode = Node(name="Model:"+model.getName())
        nodeIndex = len(self.nodes)
        self.gltf.scenes[0].nodes = [nodeIndex]
        self.nodes.append(self.modelNode)
        self.modelNodeIndex = nodeIndex;
        self.model = model;

    def addGroundFrame(self, model):
        self.groundNode = Node(name="Ground")
        nodeIndex = len(self.nodes)
        self.nodes.append(self.groundNode)
        self.modelNode.children.append(nodeIndex)
        self.mapMobilizedBodyIndexToNodes[0] = self.groundNode;
        self.mapMobilizedBodyIndexToNodeIndex[0] = nodeIndex

    def addBodyFrames(self, model):
        for body in model.getBodyList():
            nodeIndex = len(self.nodes)
            bodyNode = Node(name="Body:"+body.getAbsolutePathString())
            xform = body.getTransformInGround(self.modelState)
            t, r, s = self.createTRSFromTransform(xform, osim.Vec3(self.unitConversion))
            bodyNode.scale = s
            bodyNode.rotation = r
            bodyNode.translation = t
            self.nodes.append(bodyNode)
            self.groundNode.children.append(nodeIndex)
            self.mapMobilizedBodyIndexToNodes[body.getMobilizedBodyIndex()]=bodyNode
            self.mapMobilizedBodyIndexToNodeIndex[body.getMobilizedBodyIndex()] = nodeIndex

    def addDefaultMaterials(self):
        # create the following materials:
        # 0. default bone material for meshes
        # 1 shiny cyan material for wrap objects and contact surfaces
        # 2 shiny pink material for model markers
        # 3 shiny green material for forces
        # 4 shiny blue material for experimental markers
        # 5 shiny orange material for IMUs
        self.MaterialGrouping["ContactHalfSpace"] = "Wrapping"
        self.MaterialGrouping["ContactSphere"] = "Wrapping"
        self.MaterialGrouping["ContactMesh"] = "Wrapping"
        self.MaterialGrouping["WrapSphere"] = "Wrapping"
        self.MaterialGrouping["WrapCylinder"] = "Wrapping"
        self.MaterialGrouping["WrapEllipsoid"] = "Wrapping"
        self.MaterialGrouping["WrapTorus"] = "Wrapping"
        self.mapTypesToMaterialIndex["Mesh"] = self.addMaterialToGltf("default", [.93, .84, .77, 1.0], 0.5)
        self.mapTypesToMaterialIndex["Wrapping"] = self.addMaterialToGltf("obstacle", [0, .9, .9, 0.7], 1.0)
        self.mapTypesToMaterialIndex["Marker"] = self.addMaterialToGltf("markerMat", [1.0, .6, .8, 1.0], 1.0)
        self.mapTypesToMaterialIndex["Force"] = self.addMaterialToGltf("forceMat", [0, .9, 0, 0.7], .9)
        self.mapTypesToMaterialIndex["ExpMarker"] = self.addMaterialToGltf("expMarkerMat", [0, 0, 0.9, 1.0], 0.9)
        self.mapTypesToMaterialIndex["IMU"] = self.addMaterialToGltf("imuMat", [.8, .8, .8, 1.0], 1.0)
        

    def addMaterialToGltf(self, matName, color4, metallicFactor):
        newMaterial = Material()
        newMaterial.name = matName
        pbr = PbrMetallicRoughness()  # Use PbrMetallicRoughness
        pbr.baseColorFactor =  color4 # solid red
        pbr.metallicFactor = metallicFactor
        newMaterial.pbrMetallicRoughness = pbr
        self.materials.append(newMaterial)
        return len(self.materials)-1

    def getMaterialIndexByType(self):
        componentType = self.currentComponent.getConcreteClassName()
        materialGrouping = self.MaterialGrouping.get(componentType)
        if (materialGrouping is not None):
            mat = self.mapTypesToMaterialIndex.get(materialGrouping)
        else:
            mat = self.mapTypesToMaterialIndex.get(componentType)
        if (mat is not None):
            return mat
        else:
            return 1
 
    def setNodeTransformFromDecoration(self, node: Node, mesh: Mesh, decorativeGeometry: osim.DecorativeGeometry):
        bd = decorativeGeometry.getBodyId()
        bdNode = self.mapMobilizedBodyIndexToNodes[bd]
        nodeIndex = len(self.nodes)
        self.meshes.append(mesh)
        bdNode.children.append(nodeIndex)
        nodeForDecoration = Node(name=""+bd.getName()+decorativeGeometry.getIndexOnBody())
        self.nodes.append(nodeForDecoration)
        nodeForDecoration.matrix = self.createMatrixFromTransform(decorativeGeometry.getTransform(), decorativeGeometry.getScaleFactors())

    def createTRSFromTransform(self, xform: osim.Transform, scaleFactors: osim.Vec3):
        xr = xform.R();
        xp = xform.p();
        t = [xp.get(0), xp.get(1), xp.get(2)]
        s = [scaleFactors.get(0), scaleFactors.get(1), scaleFactors.get(2)]
        if (s[0] < 0.):
            s = [1., 1., 1.]
        q = xr.convertRotationToQuaternion();
        r = [q.get(1), q.get(2), q.get(3), q.get(0)]
        return t, r, s

    def createMatrixFromTransform(self, xform: osim.Transform, scaleFactors: osim.Vec3):
        retTransform = [1, 0, 0, 0, 0, 1, 0 , 0, 0, 0, 1, 0, 0 , 0, 0, 1];
        r = xform.R();
        p = xform.p();
        for i in range(3):
            for j in range(3):
                retTransform[i+4*j] = r.asMat33().get(i, j);
                retTransform[i+4*j] *= scaleFactors.get(j);
            retTransform[12+i] = p.get(i)*self.unitConversion;
        return retTransform

    def addMeshForPolyData(self, polyData: vtk.vtkPolyData, mat: int):
        tris = vtk.vtkTriangleFilter()
        tris.SetInputData(polyData);
        tris.Update()
        triPolys = tris.GetOutput()

        # This follows vtkGLTFExporter flow
        pointData = triPolys.GetPoints().GetData();
        self.writeBufferAndView(pointData, ARRAY_BUFFER)
        bounds = triPolys.GetPoints().GetBounds()
        # create accessor
        pointAccessor = Accessor()
        pointAccessor.bufferView= len(self.bufferViews)-1
        pointAccessor.byteOffset = 0
        pointAccessor.type = VEC3
        pointAccessor.componentType = FLOAT
        pointAccessor.count = pointData.GetNumberOfTuples()
        maxValue = [bounds[1], bounds[3], bounds[5]]
        minValue = [bounds[0], bounds[2], bounds[4]]
        pointAccessor.min = minValue
        pointAccessor.max = maxValue
        self.accessors.append(pointAccessor)
        pointAccessorIndex = len(self.accessors)-1

        # Now the normals
        normalsFilter = vtk.vtkPolyDataNormals();
        normalsFilter.SetInputData(triPolys)
        normalsFilter.ComputePointNormalsOn()
        normalsFilter.Update()
        normalsData = normalsFilter.GetOutput().GetPointData().GetNormals();
        self.writeBufferAndView(normalsData, ARRAY_BUFFER)
        normalsAccessor = Accessor()
        normalsAccessor.bufferView= len(self.bufferViews)-1
        normalsAccessor.byteOffset = 0
        normalsAccessor.type = VEC3
        normalsAccessor.componentType = FLOAT
        normalsAccessor.count = pointData.GetNumberOfTuples()
        self.accessors.append(normalsAccessor)
        normalsAccessorIndex = len(self.accessors)-1

        # now vertices
        primitive = Primitive()
        primitive.mode = 4
        if (self.processingPath):
            primitive.material = self.currentPathMaterial
        else:
            primitive.material = mat
        meshPolys = triPolys.GetPolys()
        ia = vtk.vtkUnsignedIntArray()
        idList = vtk.vtkIdList()
        while meshPolys.GetNextCell(idList):
            # do something with the cell
            for i in range(idList.GetNumberOfIds()):
                pointId = idList.GetId(i)
                ia.InsertNextValue(pointId)
        self.writeBufferAndView(ia, ELEMENT_ARRAY_BUFFER)

        indexAccessor = Accessor()
        indexAccessor.bufferView = len(self.bufferViews) - 1;
        indexAccessor.byteOffset = 0
        indexAccessor.type = SCALAR
        indexAccessor.componentType = UNSIGNED_INT
        indexAccessor.count =  meshPolys.GetNumberOfCells() * 3;
        primitive.indices = len(self.accessors)
        self.accessors.append(indexAccessor);
        primitive.attributes.POSITION= pointAccessorIndex
        primitive.attributes.NORMAL = normalsAccessorIndex
        newMesh = Mesh()
        newMesh.primitives.append(primitive)
        return newMesh;

    def writeBufferAndView(self, inData: vtk.vtkDataArray, bufferViewTarget: int):
        nt = inData.GetNumberOfTuples()
        nc = inData.GetNumberOfComponents()
        ne = inData.GetElementComponentSize()
        npArray_points = np.array(inData)
        count = nt * nc;
        byteLength = ne * count;
        encoded_result = base64.b64encode(npArray_points).decode("ascii")
        buffer = Buffer()
        buffer.byteLength = byteLength;
        buffer.uri = f"data:application/octet-stream;base64,{encoded_result}";
        self.buffers.append(buffer);
    
        bufferView = BufferView()
        bufferView.buffer = len(self.buffers)-1
        bufferView.byteOffset = 0
        bufferView.byteLength = byteLength
        bufferView.target = bufferViewTarget
        self.bufferViews.append(bufferView);

    def createGLTFLineStrip(self, point0, point1):
        linePoints = vtk.vtkPoints()
        linePoints.InsertNextPoint(point0.to_numpy())
        linePoints.InsertNextPoint(point1.to_numpy())
        line = vtk.vtkLineSource()
        line.SetPoints(linePoints)
        line.Update()
        pointData = linePoints.GetData();
        self.writeBufferAndView(pointData, ARRAY_BUFFER)
        bounds = linePoints.GetBounds()
        # create accessor
        pointAccessor = Accessor()
        pointAccessor.bufferView= len(self.bufferViews)-1
        pointAccessor.byteOffset = 0
        pointAccessor.type = VEC3
        pointAccessor.componentType = FLOAT
        pointAccessor.count = pointData.GetNumberOfTuples()
        maxValue = [bounds[1], bounds[3], bounds[5]]
        minValue = [bounds[0], bounds[2], bounds[4]]
        pointAccessor.min = minValue
        pointAccessor.max = maxValue
        self.accessors.append(pointAccessor)
        pointAccessorIndex = len(self.accessors)-1

        primitive = Primitive()
        primitive.mode = 3
        if (self.processingPath):
            primitive.material = self.currentPathMaterial
        ia = vtk.vtkUnsignedIntArray()
        ia.InsertNextValue(0)
        ia.InsertNextValue(1)
        self.writeBufferAndView(ia, ELEMENT_ARRAY_BUFFER)

        indexAccessor = Accessor()
        indexAccessor.bufferView = len(self.bufferViews) - 1;
        indexAccessor.byteOffset = 0
        indexAccessor.type = SCALAR
        indexAccessor.componentType = UNSIGNED_INT
        indexAccessor.count =  2;
        primitive.indices = len(self.accessors)
        self.accessors.append(indexAccessor);
        primitive.attributes.POSITION= pointAccessorIndex
        newMesh = Mesh()
        newMesh.primitives.append(primitive)
        return newMesh;

    def createGLTFObjectsForGeometryPath(self, geometryPath):
        """This functions creates all the artifacts needed to visualize a GeometryPath. In the case of line muscles
        These will be spheres at the PathPoints and a LineStrip for the belly of the path.
        The function also keep track of the associated nodes such that when creating an animation later, proper
        transforms will be generated as part of the animation.


        Args:
            geometryPath (_type_): _description_
        """
        if geometryPath not in self.mapPathToMaterialIndex:
            color = geometryPath.getColor(self.modelState)
            color_np = []
            for index in range(3):
                color_np.append(color.get(index))
            color_np.append(1.0)
            pathMaterialIndex = self.addMaterialToGltf(geometryPath.getAbsolutePathString()+str("Mat"), color_np, 0.5)
            self.mapPathToMaterialIndex[geometryPath] = pathMaterialIndex
            self.currentPathMaterial = pathMaterialIndex
            
        self.useTRS = True #intended so that objects created here can be animated later
        self.processingPath = True
        gPath = osim.GeometryPath.safeDownCast(geometryPath)
        self.mapPathsToNodeIds[geometryPath] = []
        currentPath = gPath.getCurrentPath(self.modelState)
        hasWrapping = geometryPath.getWrapSet().getSize() > 0
        lastPoint = currentPath.get(0)
        lastPoint.getAbsolutePathString()
        adg = osim.ArrayDecorativeGeometry()
        gPath.generateDecorations(False, self.displayHints, self.modelState, adg)
        # create a sphere for adg[0] and associate with lastPoint
        adg.getElt(0).implementGeometry(self)
        nodeId = len(self.nodes)-1
        self.mapPathsToNodeIds[geometryPath].append([nodeId, 0])
        for i in range(1, adg.size()):
            adg.getElt(i).implementGeometry(self)
            nodeId = len(self.nodes)-1
            self.mapPathsToNodeIds[geometryPath].append([nodeId, 1- i % 2])
        self.useTRS = False
        self.processingPath = False

    def createExtraAnnotations(self, gltfNode: Node):
        gltfNode.extras["opensimComponentPath"] = self.currentComponent.getAbsolutePathString()
        gltfNode.extras["opensimType"] = self.currentComponent.getConcreteClassName()

    def createAnimationForStateTimeSeries(self, 
                                          timeSeriesStorage: osim.Storage, motIndex: int, animationName=""):
        # create a timeSeriesTableVec3 of translations one column per body and
        # another timeSeriesTableQuaternion of rotation one per body
        if (animationName=="") :
            animationName = timeSeriesStorage.getName()
        if (animationName=="") :
            animationName = "Anim_"+str(motIndex)
        times = osim.ArrayDouble()
        timeSeriesStorage.getTimeColumn(times)
        timeColumn = osim.Vector(times.getAsVector())
        stateStorage = osim.Storage()
        self.model.formStateStorage(timeSeriesStorage, stateStorage, False)
        stateTraj = osim.StatesTrajectory.createFromStatesStorage(self.model, stateStorage)
        rotation_arrays = []
        translation_arrays = []
        # following maps are used to keep track of positions of anchor pathpoints
        # as well as translation/rotation/scale to transform path-segments
        pathpoint_translation_map = {}
        pathsegment_translation_map = {}
        pathsegment_rotation_map = {}
        pathsegment_scale_map = {}

        bodySet = self.model.getBodySet()
        for bodyIndex in range(bodySet.getSize()):
            rotation_arrays.append(np.zeros((times.getSize(), 4), dtype="float32"))
            translation_arrays.append(np.zeros((times.getSize(), 3), dtype="float32"))

        for nextPath in self.mapPathsToNodeIds.keys():
            pathNodes = self.mapPathsToNodeIds[nextPath]
            for pathNodeIndex in range(len(pathNodes)):
                # For a pathpoint node, we only need translation array
                # for mesh type nodes, we need translation, rotation and scale
                node_type_n_index = pathNodes[pathNodeIndex]
                if (node_type_n_index[1]==0):
                    point_translation_array = np.zeros((times.getSize(), 3), dtype="float32")
                    pathpoint_translation_map[node_type_n_index[0]]= point_translation_array
                else:
                    segment_translation_array = np.zeros((times.getSize(), 3), dtype="float32")
                    pathsegment_translation_map[node_type_n_index[0]] = segment_translation_array
                    segment_rotation_array = np.zeros((times.getSize(), 4), dtype="float32")
                    pathsegment_rotation_map[node_type_n_index[0]] = segment_rotation_array
                    segment_scale_array = np.zeros((times.getSize(), 3), dtype="float32")
                    pathsegment_scale_map[node_type_n_index[0]] = segment_scale_array


        for step in range(stateTraj.getSize()):
            nextState = stateTraj.get(step)
            self.model.realizePosition(nextState)
            for bodyIndex in range(bodySet.getSize()):
                nextBody = bodySet.get(bodyIndex)
                translation = nextBody.getPositionInGround(nextState).to_numpy()
                rotationAsSimbodyRot = nextBody.getRotationInGround(nextState)
                rotzSimbodyNotation = rotationAsSimbodyRot.convertRotationToQuaternion()
                rotation = [rotzSimbodyNotation.get(1), rotzSimbodyNotation.get(2), rotzSimbodyNotation.get(3), rotzSimbodyNotation.get(0)]
                rowTime = timeColumn[step]
                for idx in range(4):
                    rotation_arrays[bodyIndex][step, idx] = rotation[idx]
                for idx in range(3):
                    translation_arrays[bodyIndex][step, idx] = translation[idx]

            # For every path in the model, call generate decorations and populate
            # arrays for transforms to incorporate into animation
            for nextPath in self.mapPathsToNodeIds.keys():
                # print(nextPath.getAbsolutePathString())
                adg = osim.ArrayDecorativeGeometry()
                nextPath.generateDecorations(False, self.displayHints, nextState, adg)
                pathNodes = self.mapPathsToNodeIds[nextPath]
                for pathNodeIndex in range(len(pathNodes)):
                    node_type_n_index = pathNodes[pathNodeIndex]
                    next_deco_geometry = adg.getElt(pathNodeIndex)
                    newTransform = next_deco_geometry.getTransform()
                    # get new geometry from adg, add entry for translation to go from old to new
                    if (node_type_n_index[1]==0): #translation
                        for idx in range(3):
                            pathpoint_translation_map[node_type_n_index[0]][step, idx] = newTransform.p().get(idx)
        # print(pathpoint_translation_map)
        # create an Animation Node
        animation = Animation()
        if (animationName==""):
            animName = "Animation_"+str(len(self.animations)+1)
        else:
            animName = animationName
        animation.name = animName
        self.animations.append(animation)
        animationIndex = len(self.animations)-1
        # create 2 channels per body one for rotation, other for translation
        # keep track of first samplers index then create 2 per body for rotation, translation
        addTimeStampsAccessor(self.gltf, timeColumn.to_numpy())
        # this is the input to every  sampler's input
        timeAccessorIndex = len(self.gltf.accessors)-1
        # create time sampler
        for bodyIndex in range(bodySet.getSize()):
            # Create samplers
            rotSamplerIndex = len(animation.samplers)    #2*bodyIndex
            transSamplerIndex = rotSamplerIndex+1   #2*bodyIndex+1
            rotSampler = AnimationSampler()
            transSampler = AnimationSampler()
            rotSampler.input = timeAccessorIndex
            transSampler.input = timeAccessorIndex
            rotSampler.output = createAccessor(self.gltf, rotation_arrays[bodyIndex], 'r')
            transSampler.output = createAccessor(self.gltf, translation_arrays[bodyIndex], 't')
            animation.samplers.append(rotSampler)
            animation.samplers.append(transSampler)
            # Create channels
            rotChannelIndex = len(animation.channels)
            transChannelIndex = rotChannelIndex+1
            # nextChannelNumber for rotations, nextChannelNumber+1 for translations
            rotChannel = AnimationChannel()
            transChannel = AnimationChannel()
            animation.channels.append(rotChannel)
            animation.channels.append(transChannel) 

            # find target node           
            nextBody = bodySet.get(bodyIndex)
            mobodyIndex = nextBody.getMobilizedBodyIndex()
            bodyNodeIndex = self.mapMobilizedBodyIndexToNodeIndex[mobodyIndex]

            #Point channels back to samplers
            rtarget = AnimationChannelTarget()
            rtarget.node = bodyNodeIndex
            rtarget.path =  "rotation"
            rotChannel.target = rtarget
            rotChannel.sampler = rotSamplerIndex

            ttarget = AnimationChannelTarget()
            ttarget.node = bodyNodeIndex
            ttarget.path = "translation"
            transChannel.target = ttarget
            transChannel.sampler = transSamplerIndex

        for nextPath in self.mapPathsToNodeIds.keys():
            pathNodes = self.mapPathsToNodeIds[nextPath]
            for pathNodeIndex in range(len(pathNodes)):
                node_type_n_index = pathNodes[pathNodeIndex]
                if (node_type_n_index[1]==0): #translation only
                    # create Sampler, accessor and channel for each of the corresponding nodes
                    # Create samplers
                    transSamplerIndex = len(animation.samplers)
                    transSampler = AnimationSampler()
                    transSampler.input = timeAccessorIndex
                    transSampler.output = createAccessor(self.gltf, pathpoint_translation_map[node_type_n_index[0]], 't')
                    animation.samplers.append(transSampler)
                    # Create channels
                    transChannelIndex = len(animation.channels)
                    # nextChannelNumber for rotations, nextChannelNumber+1 for translations
                    transChannel = AnimationChannel()
                    animation.channels.append(transChannel) 
                    ttarget = AnimationChannelTarget()
                    ttarget.node = node_type_n_index[0]
                    ttarget.path = "translation"
                    transChannel.target = ttarget
                    transChannel.sampler = transSamplerIndex


        # Add builtin cameras
        # first create nodes for the cameras, then accessors that will be used to position/orient them
        cameraNodes = self.createCameraNodes(animationName)
        # create time sampler for the camera
        cameraTimes = np.array([timeColumn[0], timeColumn[timeColumn.size()-1]])
        addTimeStampsAccessor(self.gltf, cameraTimes)
        cameraTimeAccessorIndex = len(self.gltf.accessors)-1  
        # Camera X, Y, Z
        # now add samplers and channels for the cameras
        self.createCameraSamplersAndTargets(cameraNodes, animation, cameraTimeAccessorIndex)

    def createCameraSamplersAndTargets(self, cameraNodes, animation, cameraTimeAccessorIndex):
        cameraRotation_bbox_arrays = [[0., 0., 0., 1.0], [0., 0., 0.,1.0], 
                                 [0., 0.707, 0., 0.707], [0., 0.707, 0., 0.707],
                                 [-0.707, 0., 0., 0.707], [-0.707, 0., 0., 0.707]]
        cameraTranslation_bbox_arrays = [[-0.5, 1.0, 2.0], [0.5, 1.0, 2.0],
                                    [3., 0.5, -0.5], [3., 0.5, 0.5],
                                    [0., 3., 0.], [1., 3.0, 0.]]
        cameraRotation_arrays = []
        cameraTranslation_arrays = []
        for camIndex in range(len(cameraNodes)):
            cameraRotation_arrays.append(np.zeros((2, 4), dtype="float32"))
            cameraTranslation_arrays.append(np.zeros((2, 3), dtype="float32"))
            # for camera "cam" 
            # append cameraRotation_arrays[2*cam], cameraRotation_arrays[2*cam+1] to rotations
            # append cameraTranslation_arrays[2*cam], cameraRotation_arrays[2*cam+1] to rotations
            for step in range(2):
                for idx in range(4):
                    cameraRotation_arrays[camIndex][step, idx] = cameraRotation_bbox_arrays[2*camIndex+step][idx]
                for idx in range(3):
                    cameraTranslation_arrays[camIndex][step, idx] = cameraTranslation_bbox_arrays[2*camIndex+step][idx]
            #for every camera, will have 2 samplers one for rotations, the other for translations 
            # will ruse cameraTimeAccessorIndex
            rotSamplerIndex = len(animation.samplers)
            transSamplerIndex = rotSamplerIndex+1
            camRotSampler = AnimationSampler()
            camTransSampler = AnimationSampler()
            camRotSampler.input = cameraTimeAccessorIndex
            camTransSampler.input = cameraTimeAccessorIndex
            camRotSampler.output = createAccessor(self.gltf, cameraRotation_arrays[camIndex], 'r')
            camTransSampler.output = createAccessor(self.gltf, cameraTranslation_arrays[camIndex], 't')
            animation.samplers.append(camRotSampler)
            animation.samplers.append(camTransSampler)
            # Create channels
            camRotChannelIndex = len(animation.channels)
            camTransChannelIndex = camRotChannelIndex+1
            # nextChannelNumber for rotations, nextChannelNumber+1 for translations
            camRotChannel = AnimationChannel()
            camTransChannel = AnimationChannel()
            animation.channels.append(camRotChannel)
            animation.channels.append(camTransChannel) 
            # get camera node index
            camNodeIndex = cameraNodes[camIndex]

            rtarget = AnimationChannelTarget()
            rtarget.node = camNodeIndex
            rtarget.path =  "rotation"
            camRotChannel.target = rtarget
            camRotChannel.sampler = rotSamplerIndex
            ttarget = AnimationChannelTarget()
            ttarget.node = camNodeIndex
            ttarget.path = "translation"
            camTransChannel.target = ttarget
            camTransChannel.sampler = transSamplerIndex

    def createCameraNodes(self, animationName):
        camaraPathSuffixes = ["X", "Z", "Y"]
        cameraNodes = [];
        #create 3 scene cameras for tracking along X, Y, Z directions
        for suffix in camaraPathSuffixes:
            cameraNodes.append(addCamera(self.gltf, str("Cam"+animationName+suffix), None))
        return cameraNodes


