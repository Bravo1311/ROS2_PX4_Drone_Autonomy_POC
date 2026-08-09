import torch
import torch.nn as nn
import math

def sinusoidal_embedding(t, dim):
    """
        t: (B, ) tensor of timesteps in [0, 1] (denoising t)
        dim: embedding dimension (must be even) - half with sine, half with cosine
        returns (B, dim)
    """
    half = dim // 2

    # shape (half, )
    freqs = torch.exp(-math.log(10000) * torch.arange(half, device = t.device).float() / half)
    
    # shape (B, half)
    args = t[:, None].float() * freqs[None, :]   # t.T * freqs

    # (B, 32) + (B, 32) -> (B, 64)
    embedding = torch.cat([torch.sin(args), torch.cos(args)], dim = -1)
    
    return embedding

class AdaLN(nn.Module):
    """
        Adaptive LayerNorm: standard LayerNorm, but its scale (gamma) and shift (beta) are predicted from a conditioning vector instead of being fixed learned parameters.

        AdaLN(x, t) = Gamma(t)*(x-mu)/sigma + Beta(t); LayerNorm has fixed Gamma and Beta (learned through training, but fixed afterwards)
        Here, Gamma and Beta are outputs of a small neural network that take t-embedding as input
        gamma, beta = mlp(sinusoidal_embedding(t, dim))  both (B, num_channels)

        the network needs to behave differently at t = 0 and t = 1. Rather than implicit training based on t in the network and to hope it gets captured, AdaLN gives every layer a direct learned dial it can turn based on t
    """
    def __init__(self, d_model, cond_dim):
        # d-model: no. of channels/hidden dimention/ numbers making up each token/feature vector inside the transformer
        # cond_dim: size of the vector driving the conditioning, here t
        super().__init__()

        # no learned affine as we want pose + timestep based conditioning -> supplied
        self.norm = nn.LayerNorm(d_model, elementwise_affine = False)
        # predicts [gamma, beta]

        # weights shape: 2*d_model, cond_dim.     (reversed order): x * W.T
        self.proj = nn.Linear(cond_dim, 2 * d_model) 

    def forward(self, x, cond):
        # x: (B, seq_len, d_model), cond: (B, cond_dim)
        gamma, beta = self.proj(cond).chunk(2, dim = -1)  # each (B, d_model)
        gamma = gamma.unsqueeze(1)
        beta = beta.unsqueeze(1)
        # 1 added to prevent the signal from blowing up: training should be stable early on and 1+gamma ~ 1 initilally
        return self.norm(x) * (1 + gamma) + beta
    
class DiTBlock(nn.Module):
    """
        One transformer block: self-attention + MLP, each preceded by AdaLN conditioning, with  residual connections around both
        2 AdaLN layers give conditioing more surface area to influence the network - injecting the input at every stage
        self.attn: each position in action chunk cross-attends with each other, eg. vel3 cross attend with vel2 and vel4 to be smoother
    """

    def __init__(self, d_model, n_heads, cond_dim, mlp_ratio=4):
        super().__init__()
        self.norm1 = AdaLN(d_model, cond_dim)

        # without the flag -> mha defaults to (seq_len, B, d_model)
        self.attn = nn.MultiheadAttention(d_model, n_heads, batch_first=True)
        self.norm2 = AdaLN(d_model, cond_dim)
        self.mlp = nn.Sequential(
            nn.Linear(d_model, d_model * mlp_ratio),
            nn.GELU(),
            nn.Linear(d_model * mlp_ratio, d_model)
        )

    def forward(self, x, cond):
        # x: (B, seq_len, d_model) -> encoded action tensor
        # cond: (B, cond_dim)
        h = self.norm1(x, cond)
        attn_out, _ = self.attn(h, h, h)  # self-attention: query = key = value = h
        x = x + attn_out     # residual connection

        h = self.norm2(x, cond)
        x = x + self.mlp(h)   # residual connection
        return x

class ActionChunkEncoder(nn.Module):
    """
        Projects raw (B, C, action_dim) noisy actions into (B, C, d_model), adding a learned positional embedding per chunk-position.
    """
    def __init__(self, action_dim, d_model, chunk_len):
        super().__init__()
        self.input_proj = nn.Linear(action_dim, d_model)

        # learned tensor: one embedding vector per chunk position (8 here), shared across the batch
        self.pos_embed = nn.Parameter(torch.zeros(1, chunk_len, d_model))
        # near 0 initialization with small random noise breaks symmetry between positions right form the start. If all 8 same initially, similar gradient propagation for all for longer than necessary
        nn.init.normal_(self.pos_embed, std = 0.02)

    def forward(self, x):
        # x: (B, C, action_dim)
        x = self.input_proj(x)   # (B, C, d_model)
        x = x + self.pos_embed    # broadcast add: (1, C, d_model) + (B, C, d_model)
        return x

class HistoryEncoder(nn.Module):
    """
        Encodes (B, H, pose_dim) pose history into a single (B, cond_dim) vector
    """
    def __init__(self, pose_dim, d_model, cond_dim, history_len, n_heads=4, n_layers=2):
        super().__init__()
        self.input_proj = nn.Linear(pose_dim, d_model)
        self.pos_embed = nn.Parameter(torch.zeros(1, history_len, d_model))
        nn.init.normal(self.pos_embed, std=0.02)

        # a small plain transformer encoder - no AdaLN needed here since this module is what produces the conditioning, nothing conditions it
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=d_model, nhead=n_heads, batch_first=True, dim_feedforward=d_model*4
        )
        self.encoder = nn.TransformerEncoder(encoder_layer, num_layers=n_layers)

        self.output_proj = nn.Linear(d_model, cond_dim)

    def forward(self, history):
        # history: (B, H, pose_dim)
        x = self.input_proj(history) + self.pos_embed   # (B, H, d_model)
        x = self.encoder(x)   # (B, H, d_model)
        pooled = x.mean(dim=1)   # (B, d_model) - average over H positions
        return self.output_proj(pooled)    # (B, cond_dim)

class FlowMatchingTransformer(nn.Module):
    def __init__(self, pose_dim, action_dim, history_len, chunk_len,
            d_model=128, n_heads=4, n_block=4, cond_dim=128):
        super().__init__()
        self.d_model = d_model

        # --- condition side: history pose sequence + flow-matching timestep ---
        self.history_encoder = HistoryEncoder(
            pose_dim=pose_dim, d_model=d_model, cond_dim=cond_dim, history_len=history_len
        )
        self.time_mlp = nn.Sequential(
            nn.Linear(d_model, cond_dim),
            nn.SiLU(),
            nn.Linear(cond_dim, cond_dim),
        )
        self.time_embed_dim = d_model   # dim fed into sinusoidal_embedding

        # --- action-chunk side: what gets denoised ---
        self.action_encoder = ActionChunkEncoder(
            action_dim = action_dim, d_model=d_model, chunk_len=chunk_len
        )
        self.blocks = nn.ModuleList([
            DiTBlock(d_model=d_model, n_heads=n_heads, cond_dim=cond_dim)
            for _ in range(n_block)
        ])
        self.output_proj = nn.Linear(d_model, action_dim)

    def forward(self, noisy_actions, t, history):
        """
            noisy_actions: (B, C, action_dim) - x_t, the current noisy action chunk
            t: (B, )  - flow-matching timestep, in [0,1]
            history: (B, H, pose_dim)  - pose history condition
            returns: (B, C, action_dim)  - predicted velocity field
        """

        # --- build the combined conditioning vector ---
        hist_cond = self.history_encoder(history)   # (B, cond_dim)
        t_raw = sinusoidal_embedding(t, self.time_embed_dim)   # (B, d_model)
        t_cond = self.time_mlp(t_raw)   # (B, cond_dim)
        cond = hist_cond + t_cond   # (B, cond_dim)

        # --- process the action chunk through DiT blocks ---
        x = self.action_encoder(noisy_actions)   #(B, c, d_model)
        for block in self.blocks:
            x = block(x, cond)   # (B, C, d_model)

        return self.output_proj(x)    # (B, C, action_dim)
            