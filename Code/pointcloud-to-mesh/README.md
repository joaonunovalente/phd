# Point cloud to mesh

This project explores the possibility of converting a point cloud into a 3D mesh.

The current example is a _exhaust manifold_ point cloud.

The methods used to perform **surface reconstruction** were:
- Ball-Pivoting Algorithm
- Poisson' reconstruction


### Run code

```bash
# Create the virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install open3D
pip install open3D

# Run the example
cd CODE && ./main.py
```

