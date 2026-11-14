import sys
import os

def main():
    sys.path.append('../opensim-viewer-backend')
    #os.add_dll_directory('D:/dev/src/opensim-core-install/bin')
    import osimViewport
    import osimConverters.openSimData2Gltf
    import osimConverters.osimViewerOptions
    ovp = osimViewport.osimViewport()
    options =  osimConverters.osimViewerOptions()
    options.setShowMuscles(False)
    options.setShowMarkers(False)
    # ovp.addModelAndMotionFiles('D:/dev/src/opensim-viewer-backend/smart/leg6dof9musc.osim',
    #                            ['D:/dev/src/opensim-viewer-backend/smart/leg69_IK_stance.mot'])
    # runningGltf = ovp.getGltf();
    # options.setAddToCurrentGltf(True);
    # osimConverters.convertForceFileToGltf('D:/dev/src/opensim-viewer-backend/smart/leg69_stance_grf.mot', options, runningGltf)
    # ovp.writeGltf(runningGltf, 'D:/dev/src/opensim-viewer-backend/smart/leg39_with_forces.gltf');
    # # options.setShowMuscleColors(True)
    # this will generate a file OpenCapData_c3820ac0.gltf
    # ovp.addDataFile("C:/Users/ayman/Downloads/Walk.mot")
    # ovp.addModelFile("D:/dev/opensim-viewer-backend/LaiUhlrich2022_scaled_obj.osim")
    # ovp.saveGltf('fails.gltf')
    # # ## Model with no wrapping 
    ovp.addModelAndMotionFiles('D:/dev/src/opensim-viewer-backend/smart/LaiUhlrich2022_KA_scaled.osim', 
                                  ['D:/dev/src/opensim-viewer-backend/smart/rc_l1_trimmed.mot'], options)
    runningGltf = ovp.getGltf();
    options.setAddToCurrentGltf(True);
    #ovp.saveGltf('D:/dev/src/opensim-viewer-backend/smart/rc_l1_trimmed.gltf')
    osimConverters.convertForceFileToGltf('D:/dev/src/opensim-viewer-backend/smart/rc_l1_grf.mot', options, runningGltf)
    ovp.writeGltf(runningGltf, 'D:/dev/src/opensim-viewer-backend/smart/motion_with_forces.gltf');
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