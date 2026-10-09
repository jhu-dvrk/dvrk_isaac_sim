from dvrk_isaac_sim import simulation_worker


def test_application_closes_after_worker_acknowledgment(monkeypatch):
    events = []

    class Runtime:
        def close_application(self, status):
            events.append(("close_application", status))

    runtime = Runtime()
    monkeypatch.setattr(simulation_worker, "create_runtime", lambda start: (runtime, {}))

    def worker_main(factory):
        assert factory({}) == (runtime, {})
        events.append("stopped_ack")
        return 0

    monkeypatch.setattr(simulation_worker, "worker_main", worker_main)
    assert simulation_worker.main() == 0
    assert events == ["stopped_ack", ("close_application", 0)]
