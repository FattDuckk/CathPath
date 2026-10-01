import numpy as np
import pyvista as pv

mesh = pv.read("data/aorta.ply")
c = np.loadtxt("build/centreline.csv", delimiter=",", skiprows=1)

p = pv.Plotter()
p.add_mesh(mesh, color="lightgrey", opacity=0.15)
p.add_mesh(pv.lines_from_points(c).tube(radius=0.8), color="red")
p.add_points(c[[0, -1]], color="blue", point_size=15, render_points_as_spheres=True)  # the two ends
p.show()