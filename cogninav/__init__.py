"""CogniNav — GPU-accelerated scene understanding for vehicles and warehouse robots.

Streaming SLAM (LingBot-Map) + lane/path detection + open-vocabulary object
detection (LocateAnything-3B), fused into a single navigation dashboard.
"""

import os

# Must be set before torch initializes CUDA; avoids allocator fragmentation
# on long streaming sequences (recommended by lingbot-map).
os.environ.setdefault("PYTORCH_CUDA_ALLOC_CONF", "expandable_segments:True")

__version__ = "2.0.0"
