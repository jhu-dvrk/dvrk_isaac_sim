import json
from pathlib import Path
import subprocess
import sys
import textwrap


def test_worker_keeps_native_logs_out_of_json_responses():
    code = textwrap.dedent("""
        import os
        import sys
        from types import SimpleNamespace
        from dvrk_isaac_sim.newton_contact_worker import main

        class World:
            def __init__(self, *args, **kwargs):
                os.write(1, b'native initialization log\\n')
                print('Python initialization log')
                self.state = SimpleNamespace(body_q=SimpleNamespace(numpy=lambda: []))
                self.kinematic = {}
                self.dynamic = {}
                self.shape_paths = {}
                self.contact_count = 0
                self.active_shapes = set()

            def advance(self, elapsed, targets):
                os.write(1, b'native stepping log\\n')
                print('Python stepping log')

        sys.modules['pxr'] = SimpleNamespace(Usd=SimpleNamespace(Stage=SimpleNamespace(Open=lambda path: None)))
        sys.modules['dvrk_isaac_sim.newton_contacts'] = SimpleNamespace(NewtonContactWorld=World)
        raise SystemExit(main())
    """)
    messages = [
        {"scene": "unused.usda", "roots": [], "device": "cpu"},
        {"elapsed": 0.01, "targets": {}},
    ]
    result = subprocess.run(
        [sys.executable, "-u", "-c", code],
        input="".join(json.dumps(message) + "\n" for message in messages),
        capture_output=True,
        text=True,
        cwd=Path(__file__).resolve().parents[1],
        timeout=15,
        check=True,
    )

    responses = [json.loads(line) for line in result.stdout.splitlines()]
    assert len(responses) == 2
    assert all(response["contact_count"] == 0 for response in responses)
    assert "native initialization log" in result.stderr
    assert "native stepping log" in result.stderr
    assert "Python initialization log" in result.stderr
    assert "Python stepping log" in result.stderr