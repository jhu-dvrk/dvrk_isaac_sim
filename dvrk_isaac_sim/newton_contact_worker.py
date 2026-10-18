"""Private JSON-lines contact worker, isolated from Isaac's Python libraries."""

import contextlib
import json
import os
import sys
import traceback


def main():
    sys.stdout.flush()
    with os.fdopen(os.dup(sys.stdout.fileno()), "w", buffering=1) as output:
        os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
        return _serve(output)


def _serve(output):
    world = None
    for line in sys.stdin:
        try:
            message = json.loads(line)
            with contextlib.redirect_stdout(sys.stderr):
                if world is None:
                    from pxr import Usd
                    from .newton_contacts import NewtonContactWorld

                    stage = Usd.Stage.Open(message["scene"])
                    device = message["device"]
                    if device == "auto":
                        import warp as wp

                        device = "cuda:0" if wp.is_cuda_available() else "cpu"
                    world = NewtonContactWorld(stage, message["roots"], device=device)
                else:
                    world.advance(message["elapsed"], message["targets"])
                body_poses = world.state.body_q.numpy()
                result = {
                    "kinematic": list(world.kinematic),
                    "dynamic": list(world.dynamic),
                    "shape_paths": world.shape_paths,
                    "poses": {path: body_poses[world.body_paths[path]].tolist() for path in world.dynamic},
                    "contact_count": world.contact_count,
                    "active_shapes": list(world.active_shapes),
                }
            output.write(json.dumps(result) + "\n")
            output.flush()
        except Exception as error:
            traceback.print_exc(file=sys.stderr)
            output.write(json.dumps({"error": str(error)}) + "\n")
            output.flush()
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())