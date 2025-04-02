from pydrake.visualization import ModelVisualizer

ur_path = "./ur_description/urdf/ur3e_cylinders_collision.urdf"

modelViz = ModelVisualizer(visualize_frames=True)

modelViz.AddModels(ur_path)
modelViz.Run()
