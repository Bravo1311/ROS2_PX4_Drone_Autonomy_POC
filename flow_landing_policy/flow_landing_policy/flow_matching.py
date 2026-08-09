import torch

def flow_matching_loss(model, action_chunk, history):
    """
        action_chunk: (B, C, action_dim) - real, clean action chunks (x_1) from the dataset
        history: (B, H, pose_dim) - condition for this batch
        returns: scalar loss
        Big Picture: for each training example, pick a random point along a straight-line path between noise and the real action chunk. The network learns to predict the velocity (direction + speed) to carry that point toward the real data. -> flow matching
    """
    B = action_chunk.shape[0]
    device = action_chunk.device

    # --- sample noise (x_0) and random timestep t, per example in the batch ---
    x0 = torch.randn_like(action_chunk)   # (B, C, action_dim)

    # pick a random time from [0, 1]
    t = torch.rand(B, device = device)    # (B,) — uniform in [0,1]

    # --- build the interpolated point x_t and the target velocity ---
    t_expand = t.view(B, 1, 1)      # reshape for broadcasting over (C, action_dim)

    # xt​=(1−t)⋅x0​+t⋅x1​
    xt = (1 - t_expand) * x0 + t_expand * action_chunk   # (B, C, action_dim)
    target_velocity = action_chunk - x0    # (B, C, action_dim), constant along the path

    # --- predict velocity, compute loss ---
    predicted_velocity = model(xt, t, history)   # (B, C, action_dim)
    loss = torch.mean((predicted_velocity - target_velocity) ** 2)
    return loss