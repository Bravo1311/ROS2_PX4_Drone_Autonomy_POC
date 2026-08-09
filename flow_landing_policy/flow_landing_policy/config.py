HISTORY_LEN = 4    # H: numper of past poses (including current) fed as condition
CHUNK_LEN = 8     # C: number of future actions predicted per flow-matching pass
POSE_DIM = 7     # relative_pos(3) + relative_quat(4)
ACTION_DIM = 4     # vx, vy, vz, yaw_rate
PATIENCE = 5

DATA_DIR = "data/episodes"