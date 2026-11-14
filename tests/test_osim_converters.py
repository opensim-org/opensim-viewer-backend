"""Unit tests for the small, data-independent osimConverters helpers."""

import importlib
from pathlib import Path

import pytest

import osimConverters as osimC
from osimConverters import ViewVolume, osimViewerOptions
from osimConverters.convertOsimZip2Gltf import locateFolderContainingFile
from osimConverters.openSimData2Gltf import (
    createForceDictionary,
    getForceMeshScale,
    getForceNormalizationScale,
    getMarkerMeshScale,
    mapShapeStringToMeshNumber,
)


def test_convert_osim2gltf_initializes_model_and_returns_gltf(monkeypatch, tmp_path):
    converter = importlib.import_module("osimConverters.convertOsim2Gltf")
    model_file = tmp_path / "model.osim"
    model_file.write_text("", encoding="utf-8")
    starting_gltf = object()
    display_hints = type(
        "DisplayHints",
        (),
        {
            "set_show_markers": lambda self, value: setattr(
                self, "show_markers", value
            ),
            "set_show_frames": lambda self, value: setattr(
                self, "show_frames", value
            ),
        },
    )()

    class Model:
        def initSystem(self):
            return "model-state"

        def getDisplayHints(self):
            return display_hints

        def getGround(self):
            return "ground"

        def getComponentsList(self):
            return []

    class GeometryImplementation:
        def init(self):
            pass

        def setGltf(self, gltf):
            self.gltf = gltf

        def setState(self, state):
            self.state = state

        def setDisplayHints(self, hints):
            self.display_hints = hints

        def addModelNode(self, model):
            self.model = model

        def addGroundFrame(self, ground):
            self.ground = ground

        def addBodyFrames(self, model):
            pass

        def addDefaultMaterials(self):
            pass

        def get_GLTF(self):
            return self.gltf

    geometry_paths = []
    monkeypatch.setattr(
        converter.osim,
        "ModelVisualizer",
        type(
            "ModelVisualizer",
            (),
            {"addDirToGeometrySearchPaths": staticmethod(geometry_paths.append)},
        ),
    )
    monkeypatch.setattr(converter.osim, "Model", lambda path: Model())
    monkeypatch.setattr(
        converter.osim, "ArrayDecorativeGeometry", lambda: object()
    )
    monkeypatch.setattr(
        converter,
        "DecorativeGeometryImplementationGltf",
        GeometryImplementation,
    )
    options = type(
        "Options",
        (),
        {
            "getShowMarkers": lambda self: False,
            "getShowMuscles": lambda self: True,
        },
    )()

    result = converter.convertOsim2Gltf(
        str(model_file), "custom-geometry", options=options, startingGltf=starting_gltf
    )

    assert result is starting_gltf
    assert geometry_paths[0] == "custom-geometry"
    assert len(geometry_paths) == 2  # requested geometry plus bundled fallback
    assert display_hints.show_markers is False
    assert display_hints.show_frames is True


def test_convert_osim2gltf_double_pendulum_regression():
    converter = importlib.import_module("osimConverters.convertOsim2Gltf")
    model_file = Path(__file__).with_name("double_pendulum.osim")
    options = osimViewerOptions()
    options.setShowMuscles(False)

    gltf = converter.convertOsim2Gltf(
        str(model_file), "", options=options
    )

    node_names = {node.name for node in gltf.nodes}
    assert "Model:double_pendulum" in node_names
    assert "Ground" in node_names
    assert any(name.startswith("Body:/bodyset/rod1") for name in node_names)
    assert any(name.startswith("Body:/bodyset/rod2") for name in node_names)
    assert gltf.scenes[0].nodes == [0]


def test_leg6dof9musc_motion_and_ground_reaction_force_regression():
    fixture_dir = Path(__file__).parent
    model_file = fixture_dir / "leg6dof9musc.osim"
    motion_file = fixture_dir / "leg69_IK_stance.mot"
    force_file = fixture_dir / "leg69_stance_grf.mot"
    options = osimViewerOptions()
    options.setShowMuscles(False)
    options.setShowMarkers(False)

    gltf = osimC.convertNativeFileSetToGLTF(
        str(model_file), [str(motion_file)], options
    )
    assert gltf.nodes[0].name == "Model:leg6dof9musc"
    assert gltf.animations
    model_animation_channel_count = sum(
        len(animation.channels) for animation in gltf.animations
    )
    assert model_animation_channel_count > 0

    gltf_with_forces = osimC.convertForceFileToGltf(
        str(force_file), options, gltf
    )

    assert gltf_with_forces is gltf
    force_data_node = next(
        node for node in gltf.nodes if node.name == "ForceData"
    )
    assert force_data_node.children
    assert sum(len(animation.channels) for animation in gltf.animations) > (
        model_animation_channel_count
    )
    gltf.save("leg6dof9musc.gltf")


def test_convert_trc2gltf_converts_marker_table(monkeypatch, tmp_path):
    converter = importlib.import_module("osimConverters.convertTrc2Gltf")
    trc_file = tmp_path / "markers.trc"
    trc_file.write_text("", encoding="utf-8")
    gltf = object()
    marker_table = object()
    calls = []
    monkeypatch.setattr(converter, "initGltf", lambda: gltf)
    monkeypatch.setattr(
        converter.osim, "TimeSeriesTableVec3", lambda path: marker_table
    )
    monkeypatch.setattr(
        converter,
        "convertMarkersTimeSeries2Gltf",
        lambda target, shape, table: calls.append((target, shape, table)),
    )
    options = type(
        "Options", (), {"getExperimentalMarkerShape": lambda self: "sphere"}
    )()

    result = converter.convertTrc2Gltf(str(trc_file), options)

    assert result is gltf
    assert calls == [(gltf, "sphere", marker_table)]


def test_convert_trc2gltf_arm26_elbow_flex_regression():
    converter = importlib.import_module("osimConverters.convertTrc2Gltf")
    trc_file = Path(__file__).with_name("arm26_elbow_flex.trc")

    gltf = converter.convertTrc2Gltf(str(trc_file), osimViewerOptions())

    assert gltf.scenes[0].nodes == [0]
    assert gltf.nodes[0].name == "MarkerData"
    markers = gltf.nodes[1:4]
    assert [node.name for node in markers] == [
        "r_acromion",
        "r_humerus_epicondyle",
        "r_radius_styloid",
    ]
    # The TRC coordinates are in millimeters and the converter stores meters.
    assert markers[0].translation == pytest.approx(
        [-0.01305452443, 0.83950547557, 0.1695054756]
    )
    assert len(gltf.animations) == 1
    assert len(gltf.animations[0].channels) == 3
    assert all(
        channel.target.path == "translation"
        for channel in gltf.animations[0].channels
    )


@pytest.mark.parametrize(
    ("shape", "mesh_index"),
    [("brick", 0), ("sphere", 1), ("cube", 2), ("arrow", 3)],
)
def test_map_shape_name_to_mesh_index(shape, mesh_index):
    assert mapShapeStringToMeshNumber(shape) == mesh_index


def test_map_shape_name_rejects_unknown_shape():
    with pytest.raises(KeyError):
        mapShapeStringToMeshNumber("pyramid")


def test_create_force_dictionary_supports_opensim_vector_labels():
    forces = {}

    createForceDictionary(
        ["ground_force_v", "ground_force_p", "unused"], forces
    )

    assert forces == {"ground_force": 0}


def test_create_force_dictionary_supports_force_and_point_labels():
    forces = {}

    createForceDictionary(["ground_fx", "ground_px", "other"], forces)

    assert forces == {"ground_fx": 0}


def test_converter_scale_constants():
    assert getMarkerMeshScale() == pytest.approx(0.01)
    assert getForceMeshScale() == pytest.approx(0.2)
    assert getForceNormalizationScale() == pytest.approx(0.001)


def test_view_volume_expands_to_include_accessor_bounds():
    volume = ViewVolume("scene")
    volume.updateFromAccessor(
        type("AccessorBounds", (), {"min": [-2, 1, -3], "max": [4, 5, 6]})()
    )
    volume.updateFromAccessor(
        type("AccessorBounds", (), {"min": [-1, -4, -2], "max": [2, 7, 3]})()
    )

    assert volume.name == "scene"
    assert volume.volumeMin == [-2, -4, -3]
    assert volume.volumeMax == [4, 7, 6]


def test_viewer_options_defaults_and_setters():
    options = osimViewerOptions()

    assert options.getExperimentalMarkerShape() == "sphere"
    assert options.getExperimentalMarkerColor() == ""
    assert options.getForceShape() == "arrow"
    assert options.getShowMuscles() is True
    assert options.getShowMarkers() is True
    assert options.getAddToCurrentGltf() is False

    options.setExperimentalMarkerShape("cube")
    options.setExperimentalMarkerColor([1, 0, 0])
    options.setForceShape("brick")
    options.setShowMuscles(False)
    options.setShowMarkers(False)
    options.setAddToCurrentGltf(True)

    assert options.getExperimentalMarkerShape() == "cube"
    assert options.getExperimentalMarkerColor() == [1, 0, 0]
    assert options.getForceShape() == "brick"
    assert options.getShowMuscles() is False
    assert options.getShowMarkers() is False
    assert options.getAddToCurrentGltf() is True


def test_locate_folder_containing_file_in_nested_directory(tmp_path):
    expected = tmp_path / "session" / "OpenSimData"
    expected.mkdir(parents=True)
    (expected / "opensim-viewer.json").write_text("{}", encoding="utf-8")

    assert (
        locateFolderContainingFile(str(tmp_path), "opensim-viewer.json")
        == str(expected)
    )


def test_locate_folder_containing_file_returns_none_when_missing(tmp_path):
    assert locateFolderContainingFile(str(tmp_path), "missing.json") is None
