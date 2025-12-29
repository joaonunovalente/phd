# Experiment Vibrometer + ORBBEC FEMTO

## Create Container
``` bash
sh docker_run.sh CODE_PATH DATA_PATH

sh docker_run.sh /home/valente/Documents/vscode/PhD/Code/point-cloud-registration/pipeline-multiview-point-cloud-acquisition/exhaustive-grid-search /home/valente/Documents/vscode/PhD/Code/point-cloud-registration/pipeline-multiview-point-cloud-acquisition/data
```

## Enter docker container
```bash
docker start egs-container
docker exec -it egs-container /bin/bash
cd exhaustive-grid-search/
```

## Pipeline
``` bash
cd /home/valente/Documents/vscode/PhD/Code/point-cloud-registration/pipeline-multiview-point-cloud-acquisition/code

source .venv/bin/activate

# Capture point clouds
./capture_point_clouds.py

# Register point clouds
python demo.py  --pc_source_path ../data/test1/depth_1.ply --pc_target_path ../data/test1/depth_2.ply

# Visualize the point clouds
./visualize_registration.py 1 1 2
```

## Remove results
``` bash
cd results && rm -r test1
```