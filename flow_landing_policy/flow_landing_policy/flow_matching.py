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


def flow_matching_loss(model, action_chunk, history):
    """
    Compute the flow-matching loss for one training batch.

    Big picture: for each training example, pick a random point along a
    straight-line path between noise and the real action chunk. The network
    learns to predict the velocity (direction + speed) to carry that point
    toward the real data -> flow matching.

    action_chunk: (B, C, action_dim) - real, clean action chunks (x_1) from the dataset
    history: (B, H, pose_dim) - condition for this batch
    returns: scalar loss
    """
    B = action_chunk.shape[0]
    device = action_chunk.device

    # --- sample noise (x_0) and random timestep t, per example in the batch ---
    x0 = torch.randn_like(action_chunk)  # (B, C, action_dim)

    # pick a random time from [0, 1]
    t = torch.rand(B, device=device)  # (B,) — uniform in [0,1]

    # --- build the interpolated point x_t and the target velocity ---
    t_expand = t.view(B, 1, 1)  # reshape for broadcasting over (C, action_dim)

    # xt​=(1−t)⋅x0​+t⋅x1​
    xt = (1 - t_expand) * x0 + t_expand * action_chunk  # (B, C, action_dim)
    target_velocity = action_chunk - x0  # (B, C, action_dim), constant along the path

    # --- predict velocity, compute loss ---
    predicted_velocity = model(xt, t, history)  # (B, C, action_dim)
    loss = torch.mean((predicted_velocity - target_velocity) ** 2)
    return loss
