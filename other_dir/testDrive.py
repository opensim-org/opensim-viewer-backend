import sys
import os


def main():
    sys.path.append('../opensim-viewer-backend')
    #os.add_dll_directory('D:/dev/src/opensim-core-install/bin')
    import osimViewport
    import osimConverters.openSimData2Gltf
    import osimViewerOptions
    ovp = osimViewport.osimViewport()
    options =  osimViewerOptions.osimViewerOptions()
    # options.setShowMuscles(True)
    # options.setShowMuscleColors(True)
    # this will generate a file OpenCapData_c3820ac0.gltf
    # ovp.addDataFile("C:/Users/ayman/Downloads/Walk.mot")
    # ovp.addModelFile("D:/dev/opensim-viewer-backend/LaiUhlrich2022_scaled_obj.osim")
    # ovp.saveGltf('fails.gltf')
    # # ## Model with no wrapping 
    ovp.addModelAndMotionFiles('D:/OpenSimTestingFiles/OpenSimTestingFiles/RajagopalLaiUhlrich2023_scaled.osim', 
                                  ['D:/OpenSimTestingFiles/OpenSimTestingFiles/ik_results.sto'])
    ovp.saveGltf('RajagopalLaiUhlrich2023_scaled.gltf')
    # # ovp3 = osimViewport.osimViewport()
    # ovp.addDataFile('D:/DemoFiles/subject01_walk.trc')
    # ovp.saveGltf('walkColors.gltf')
    # # ovp2 = osimViewport.osimViewport()
    # # ovp2.addModelFile("D:/dev/opensim-viewer-backend/BuiltinGeometry.osim")
    # # ovp2.saveGltf()


    # # ovp.addModelAndMotionFiles("D:/dev/opensim-viewer-backend/arm26.osim",
    # #                           ["D:/dev/opensim-viewer-backend/arm26_flex.sto"])
    # # ovp2 = osimViewport.osimViewport()
    # # # ovp2.addModelAndMotionFiles("D:/CMBBE2024/Demo2_OpenSimIKPipeline/LaiUhlrich2022_scaled.osim",
    # # #                              ["D:/CMBBE2024/Demo2_OpenSimIKPipeline/ik.mot"])
    # # # ovp2.saveGltf()
main()