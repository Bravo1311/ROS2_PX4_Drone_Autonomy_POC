# Copyright 2026 Kartik Agrawal
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import torch
from flow_landing_policy.transformer import FlowMatchingTransformer
from flow_landing_policy.config import *

def load_policy(checkpoint_path, device):
    model = FlowMatchingTransformer(
        pose_dim=POSE_DIM, action_dim=ACTION_DIM,
        history_len=HISTORY_LEN, chunk_len=CHUNK_LEN,
    ).to(device)
    model.load_state_dict(torch.load(checkpoint_path, map_location=device))
    model.eval()
    return model

@torch.no_grad()
def generate_action_chunk(model, history, device, n_steps = 10):
    """
        history: real pose history, not batched (H, pose_dim)
        returns: (C, action_dim) numpy array - generated action chunk
    """
    # (1, H, pose_dim)
    history = torch.as_tensor(history, dtype = torch.float32, device = device).unsqueeze(0)

    x = torch.randn(1, CHUNK_LEN, ACTION_DIM, device=device)   # start from pure noise

    dt = 1.0 / n_steps
    for step in range(n_steps):
        t_val = step / n_steps
        # (1, ) - same t for this whole step
        t = torch.full((1,), t_val, device=device)

        velocity = model(x, t, history)  # (1, C, action_dim)
        x = x + velocity * dt  # Euler step

    return x.squeeze(0).cpu().numpy()
