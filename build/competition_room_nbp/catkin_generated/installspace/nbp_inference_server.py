#!/home/lin/miniforge3/envs/omnifit/bin/python
"""Persistent binary-pipe inference server for the official NBP checkpoint."""

import argparse
import os
import struct
import sys
import time

import numpy as np
import torch

PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from competition_room_nbp.official_model import NBP


INPUT_SHAPE = (1, 5, 256, 256)
OUTPUT_SHAPE = (8, 64, 64)
INPUT_BYTES = int(np.prod(INPUT_SHAPE)) * 4


def read_exact(stream, size):
    chunks = []
    remaining = size
    while remaining:
        chunk = stream.read(remaining)
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", required=True)
    parser.add_argument("--device", default="cuda")
    args = parser.parse_args()

    if args.device.startswith("cuda") and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but unavailable")
    device = torch.device(args.device)
    model = NBP()
    checkpoint = torch.load(args.weights, map_location="cpu", weights_only=False)
    state = checkpoint.get("model_state_dict", checkpoint)
    model.load_state_dict(state)
    model.eval().to(device)
    del checkpoint, state
    if device.type == "cuda":
        torch.cuda.synchronize(device)

    source = sys.stdin.buffer
    sink = sys.stdout.buffer
    sink.write(b"NBPR")
    sink.flush()
    while True:
        magic = read_exact(source, 4)
        if magic is None:
            return
        if magic != b"NBP1":
            raise RuntimeError("invalid request header")
        payload = read_exact(source, INPUT_BYTES)
        if payload is None:
            return
        tensor = torch.from_numpy(
            np.frombuffer(payload, dtype=np.float32).reshape(INPUT_SHAPE).copy()
        ).to(device)
        total_start = time.perf_counter()
        model_start = time.perf_counter()
        with torch.inference_mode():
            value_map, _ = model(tensor)
        if device.type == "cuda":
            torch.cuda.synchronize(device)
        model_ms = (time.perf_counter() - model_start) * 1000.0
        values = value_map[0].detach().float().cpu().numpy()
        total_ms = (time.perf_counter() - total_start) * 1000.0
        sink.write(struct.pack("<4sddI", b"NBPO", total_ms, model_ms, values.size))
        sink.write(values.astype(np.float32, copy=False).tobytes(order="C"))
        sink.flush()


if __name__ == "__main__":
    main()

