from scipy.io import loadmat
d = loadmat("data/Mesh_Soft_EM_Mod_0601.mat", squeeze_me=True, struct_as_record=False)
for k, v in d.items():
    if not k.startswith("__"):
        print(k, type(v), getattr(v, "shape", ""), getattr(v, "_fieldnames", ""))

import numpy as np, pyvista as pv
V, F = d["ver"], d["tri"] - 1
mesh = pv.PolyData(V, np.c_[np.full(len(F), 3), F])
mesh.plot(opacity=0.4, show_edges=False)