#!/usr/bin/env python3
"""Export the published HDPlanner policy checkpoint as TorchScript."""

import argparse
import hashlib
import json
import os
import sys

import torch


NODE_COUNT = 360
NODE_FEATURES = 7
NEIGHBOR_COUNT = 25


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def make_inputs(seed):
    generator = torch.Generator(device="cpu")
    generator.manual_seed(seed)
    node_inputs = torch.rand((1, NODE_COUNT, NODE_FEATURES), generator=generator)
    edge_inputs = torch.tensor(
        [0, 1, 2, 3, 4] + [0] * (NEIGHBOR_COUNT - 5),
        dtype=torch.int64,
    ).reshape(1, NEIGHBOR_COUNT, 1)
    current_index = torch.tensor([[[0]]], dtype=torch.int64)
    target_index = torch.tensor([[[4]]], dtype=torch.int64)
    center_inputs = torch.tensor(
        [1, 2, 3, 4] + [NODE_COUNT - 1] * (NEIGHBOR_COUNT - 4),
        dtype=torch.int64,
    ).reshape(1, 1, NEIGHBOR_COUNT)
    node_padding = torch.ones((1, 1, NODE_COUNT), dtype=torch.int16)
    node_padding[:, :, :5] = 0
    edge_padding = torch.ones((1, 1, NEIGHBOR_COUNT), dtype=torch.int16)
    edge_padding[:, :, :5] = 0
    edge_padding[:, :, 0] = 1
    edge_mask = torch.ones((1, NODE_COUNT, NODE_COUNT), dtype=torch.int64)
    edge_mask[:, :5, :5] = 0
    center_padding = torch.ones((1, 1, NEIGHBOR_COUNT), dtype=torch.int64)
    center_padding[:, :, :4] = 0
    return (
        node_inputs,
        edge_inputs,
        current_index,
        target_index,
        center_inputs,
        node_padding,
        edge_padding,
        edge_mask,
        center_padding,
    )


def clone_inputs(inputs):
    return tuple(value.clone() for value in inputs)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--metadata", required=True)
    args = parser.parse_args()

    repo = os.path.abspath(args.repo)
    sys.path.insert(0, repo)
    from model import PolicyNet

    checkpoint = torch.load(args.checkpoint, map_location="cpu", weights_only=False)
    state = checkpoint["policy_model"]
    model = PolicyNet(NODE_FEATURES, 128)
    model.load_state_dict(state, strict=True)
    model.eval()

    example_inputs = make_inputs(7)
    with torch.inference_mode():
        traced = torch.jit.trace(
            model,
            clone_inputs(example_inputs),
            check_trace=False,
            strict=True,
        )
        traced = torch.jit.freeze(traced.eval())
        eager_output = model(*clone_inputs(example_inputs))
        traced_output = traced(*clone_inputs(example_inputs))

    max_abs_error = max(
        float((expected - actual).abs().max().item())
        for expected, actual in zip(eager_output, traced_output)
    )
    if max_abs_error > 1e-5:
        raise RuntimeError("TorchScript mismatch: max_abs_error=%g" % max_abs_error)
    if int(eager_output[3].item()) != int(traced_output[3].item()):
        raise RuntimeError("TorchScript selected a different action")

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    traced.save(args.output)
    metadata = {
        "source_repository": "https://github.com/marmotlab/HDPlanner_Exp_and_Nav",
        "checkpoint": os.path.abspath(args.checkpoint),
        "checkpoint_sha256": sha256(args.checkpoint),
        "checkpoint_episode": int(checkpoint.get("episode", -1)),
        "torch_version": torch.__version__,
        "model": "PolicyNet(7, 128)",
        "node_count": NODE_COUNT,
        "node_feature_count": NODE_FEATURES,
        "neighbor_count": NEIGHBOR_COUNT,
        "max_abs_error": max_abs_error,
        "selected_action_node": int(traced_output[3].item()),
    }
    with open(args.metadata, "w", encoding="utf-8") as stream:
        json.dump(metadata, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(metadata, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
