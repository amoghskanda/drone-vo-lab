import sys

import pytest

from drone_vo_lab import benchmark, cli


def test_cli_forwards_recovery_limit(monkeypatch, tmp_path):
    received = {}
    monkeypatch.setattr(benchmark, "run_benchmark", lambda **kwargs: received.update(kwargs))
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "drone-vo",
            "benchmark",
            "--quick",
            "--no-video",
            "--max-recovery-failures",
            "2",
            "--output",
            str(tmp_path),
        ],
    )
    cli.main()
    assert received["max_recovery_failures"] == 2
    assert received["frames"] == 90
    assert received["scenes"] == ["warehouse"]
    assert received["video"] is False


@pytest.mark.parametrize("limit", ["0", "-1"])
def test_cli_rejects_nonpositive_recovery_limit(monkeypatch, capsys, limit):
    monkeypatch.setattr(sys, "argv", ["drone-vo", "benchmark", "--max-recovery-failures", limit])
    with pytest.raises(SystemExit) as error:
        cli.main()
    assert error.value.code == 2
    assert "max recovery failures must be a positive integer" in capsys.readouterr().err
