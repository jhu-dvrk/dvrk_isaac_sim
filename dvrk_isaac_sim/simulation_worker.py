from dvrk_simulator_base.command_mailbox import CommandMailboxes
from dvrk_simulator_base.process_worker import worker_main

def create_runtime(start):
    from .camera import CameraOptions
    from .configuration import load_installed_scene_config, load_simulator_config
    from .runtime import IsaacSimRuntime, RuntimeOptions

    config = load_simulator_config(start["config"])
    scene = load_installed_scene_config(start["scene"])
    commands = {item.name: CommandMailboxes(config.command_queue_capacity) for item in scene.robots}
    runtime = IsaacSimRuntime(
        scene.robots,
        RuntimeOptions(
            headless=start["headless"],
            renderer=config.renderer,
            simulation_rate_hz=config.simulation_rate_hz,
            render_rate_hz=config.render_rate_hz,
            generated_root=config.generated_root,
            camera_options=CameraOptions.from_scene(scene.camera) if scene.camera else None,
        ),
        commands,
        scene_config=scene,
    )
    return runtime, commands


def main():
    runtime = None
    status = 1

    def factory(start):
        nonlocal runtime
        runtime, commands = create_runtime(start)
        return runtime, commands

    try:
        status = worker_main(factory)
        return status
    finally:
        if runtime is not None:
            runtime.close_application(status)


if __name__ == "__main__":
    raise SystemExit(main())
