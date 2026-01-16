import os
from pathlib import Path
import zipfile
import osimConverters as osimC

def convertOpenCapZip2Gltf(zipFilePath):
    path = Path(zipFilePath)
    if not path.exists():
        raise NotADirectoryError("Unable to find file ", path.absolute())
    folderName = zipFilePath.replace('.zip', '/')
    with zipfile.ZipFile(zipFilePath, 'r') as zip_ref:
        zip_ref.extractall(folderName)
    # expected layout is 
    # folder 
    #    OpenSimData
    #       Model
    #         [model].osim
    #       Kinematics
    #          [session1].mot
    #          [session...].mot
    opensimFolderPath = None
    modelFiles = []
    motionFiles = []
    for dirpath, dirnames, filenames in os.walk(folderName):
        if ('OpenSimData' in dirnames):
            opensimFolderPath = dirpath
        for file in filenames:
            if file.endswith(".osim"):
                modelFiles.append(os.path.join(dirpath, file))
            if file.endswith(".mot"):
                motionFiles.append(os.path.join(dirpath, file))

    # if not found bail out otherwise process files
    if opensimFolderPath is not None:
        gltfOutput = osimC.convertNativeFileSetToGLTF(modelFiles[0], motionFiles)
        gltfOutput.save("opencapOutput.gltf")
        return gltfOutput

