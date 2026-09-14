"""Compare the real C++ SDK with the checkpoint's saved gait implementation.

python test_gait_observation.py --probe /path/to/rl_gait_observation_probe \
    --run /path/to/run --roboduet-root /path/to/RoboDuet \
    --config /path/to/policy/config.yaml
"""
import argparse
import ast
import json
import pickle
import subprocess
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import torch
import yaml


def namespace(value):
    return SimpleNamespace(**{k: namespace(v) for k, v in value.items()}) if isinstance(value, dict) else value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", type=Path, required=True)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--roboduet-root", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    sys.path.insert(0, str(args.roboduet_root / "scripts"))
    from export_rl_sar import dog_command_layout
    from rl_sar_obs import RlSarObservation, effective_gait_frequency

    key, bundle = next(iter(yaml.safe_load(args.config.read_text()).items()))
    cfg = namespace(pickle.loads((args.run / "parameters.pkl").read_bytes())["Cfg"])
    extra, _ = dog_command_layout(cfg)
    expected_height = sum(cfg.commands.limit_footswing_height) / 2
    assert extra[1] == expected_height
    assert bundle["dog_commands_extra"][1] == expected_height
    source = args.run / "scripts/legged_robot.py"
    method = next(n for n in ast.walk(ast.parse(source.read_text()))
                  if isinstance(n, ast.FunctionDef) and n.name == "_step_contact_targets")
    scope = {"torch": torch, "np": np}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), scope)

    commands = [(0, 0, 0), (.099, 0, 0), (.1, 0, 0), (.101, 0, 0),
                (-.099, 0, 0), (-.101, 0, 0), (0, .15, 0), (0, 0, .12), (0, 0, .099)]
    rows = []
    for mode in (1, -1, 0):  # explicit dynamic, old bundle inference, fixed gait
        for phase in np.linspace(0, .999, 101):
            for command in commands:
                rows.append([mode, phase, 2.0 if mode else 3.0, .49 if mode else .5, *command])
    # Walk -> stand held for several observations -> reverse/restart -> turn.
    for mode in (1, -1):
        rows.append([mode, .37, 2.5, .49, .4, 0, 0])
        for command in [(.04, 0, 0)] * 20 + [(0, 0, 0)] * 20 + [(-.4, 0, 0), (0, 0, .2)]:
            rows.append([mode, -1, 2.5, .49, *command])

    with tempfile.TemporaryDirectory() as directory:
        output = Path(directory) / "probe.txt"
        subprocess.run([str(args.probe), key, str(output)],
                       input="\n".join(" ".join(map(str, row)) for row in rows) + "\n",
                       text=True, check=True, stdout=subprocess.DEVNULL)
        actual = np.loadtxt(output)
    assert actual.shape == (len(rows), 17)
    worst_clock = worst_phase = worst_python = 0.0
    saved_phase = torch.zeros(1)
    for row, observed in zip(rows, actual):
        mode, phase, frequency, duration, vx, vy, yaw = row
        cfg.commands.use_dynamic_gait = bool(mode)
        if phase >= 0:
            saved_phase[:] = phase
        command = torch.tensor([[vx, vy, yaw, 0, 0, 0, frequency, expected_height,
                                 extra[2], extra[3], duration]], dtype=torch.float32)
        standing = torch.norm(command[:, :3], dim=1).item() < .1
        if mode and standing:
            command[:, 6] = 0  # the saved training sampler's standing override
        env = SimpleNamespace(cfg=cfg, commands_dog=command, gait_indices=saved_phase.clone(), dt=.02,
                              clock_inputs=torch.zeros(1, 4), doubletime_clock_inputs=torch.zeros(1, 4),
                              halftime_clock_inputs=torch.zeros(1, 4), desired_contact_states=torch.zeros(1, 4))
        scope["_step_contact_targets"](env)
        saved_phase = env.gait_indices
        expected = np.r_[command.numpy().ravel() * bundle["dog_commands_scale"], env.clock_inputs.numpy().ravel()]
        np.testing.assert_allclose(observed[2:], expected, atol=3e-5, rtol=1e-5)
        np.testing.assert_allclose(observed[0], saved_phase.item(), atol=3e-6)
        assert observed[1] == frequency, "standing must not overwrite the walking setting"
        worst_clock = max(worst_clock, float(np.abs(observed[-4:] - expected[-4:]).max()))
        worst_phase = max(worst_phase, abs(observed[0] - saved_phase.item()))

        params = dict(bundle, gait_frequency=frequency, gait_duration=duration,
                      use_dynamic_gait=bool(mode))
        params["dog_commands_extra"] = [frequency, expected_height, extra[2], extra[3], duration]
        if mode < 0:
            params.pop("use_dynamic_gait")
        builder = RlSarObservation(params)
        state = dict(cmd_x=vx, cmd_y=vy, cmd_yaw=yaw, cmd_pitch=0, cmd_roll=0, cmd_height=0,
                     gait_indices=saved_phase.item())
        replay = np.r_[builder.term("roboduet/dog_commands", state), builder.term("roboduet/clock_inputs", state)]
        np.testing.assert_allclose(replay, expected, atol=3e-5, rtol=1e-5)
        assert effective_gait_frequency(params, [vx, vy, yaw]) == command[0, 6].item()
        worst_python = max(worst_python, float(np.abs(replay - expected).max()))
    print(json.dumps(dict(cases=len(rows), max_clock_error=worst_clock, max_phase_error=worst_phase,
                          python_mirror_max_error=worst_python, footswing_height_m=expected_height), indent=2))


if __name__ == "__main__":
    main()
