#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Đóng gói kịch bản chuẩn Track D vào thư mục CSV cho MATLAB (SII/SGI GT).

Robot trajectory lấy từ bag/CSV (AMCL thật). Người/nhóm/inter/emotion lấy từ
social_scenario_canonical_trackD_real.yaml — không dùng detect cảm biến.
"""
from __future__ import annotations

import argparse
import math
import os
import shutil
from pathlib import Path

import yaml


def _yaw_quat(yaw: float):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_dir", type=Path)
    ap.add_argument("--exp-name", default="")
    ap.add_argument("--exp-variant", default="")
    ap.add_argument(
        "--canonical",
        type=Path,
        default=None,
        help="social_scenario_canonical_trackD_real.yaml",
    )
    ap.add_argument(
        "--matlab-yaml",
        type=Path,
        default=None,
        help="social_scenario_matlab_trackD_real.yaml (rút gọn cho parse_social_yaml_config.m)",
    )
    ap.add_argument("--wall", type=Path, default=None)
    args = ap.parse_args()

    csv_dir = args.csv_dir.resolve()
    csv_dir.mkdir(parents=True, exist_ok=True)

    pkg_cfg = Path(
        os.environ.get(
            "TRACK_D_SCENARIO_DIR",
            str(
                Path(__file__).resolve().parents[1]
                / "src"
                / "social_density_fusion"
                / "config"
                / "D-density-costmap"
                / "real"
            ),
        )
    )
    # When run from workspace scripts/, parents[1] is omni4wd_ws
    ws_cfg = (
        Path(__file__).resolve().parents[1]
        / "src"
        / "social_density_fusion"
        / "config"
        / "D-density-costmap"
        / "real"
    )
    if not (args.canonical or (ws_cfg / "social_scenario_canonical_trackD_real.yaml").is_file()):
        # fallback: script in scripts/ → ws/src/...
        pass

    canonical = args.canonical or (ws_cfg / "social_scenario_canonical_trackD_real.yaml")
    matlab_yaml = args.matlab_yaml or (ws_cfg / "social_scenario_matlab_trackD_real.yaml")
    if not canonical.is_file():
        raise SystemExit(f"Missing canonical: {canonical}")
    if not matlab_yaml.is_file():
        raise SystemExit(f"Missing matlab yaml: {matlab_yaml}")

    data = yaml.safe_load(canonical.read_text(encoding="utf-8")) or {}

    # 1) social_scenario.yaml — bản MATLAB parser dùng
    shutil.copy2(matlab_yaml, csv_dir / "social_scenario.yaml")
    shutil.copy2(canonical, csv_dir / "social_scenario_canonical.yaml")

    # 2) environment_objects.txt
    obj_lines = ["model_name object_type x y z yaw"]
    for obj in data.get("objects") or []:
        if not isinstance(obj, dict):
            continue
        obj_lines.append(
            "{name} picture {x:.6f} {y:.6f} {z:.6f} 0.000000".format(
                name=obj.get("name", "object"),
                x=float(obj.get("x", 0.0)),
                y=float(obj.get("y", 0.0)),
                z=float(obj.get("z", 0.0)),
            )
        )
    (csv_dir / "environment_objects.txt").write_text("\n".join(obj_lines) + "\n", encoding="utf-8")

    # 3) wall
    wall = args.wall
    if wall and wall.is_file():
        shutil.copy2(wall, csv_dir / "wall_segments.txt")

    # 4) static_actors_snapshot.csv (actor1..13 static; 14/15 pedestrian)
    actors = [a for a in (data.get("actors") or []) if isinstance(a, dict) and "name" in a]
    actors_sorted = sorted(
        actors,
        key=lambda a: int("".join(ch for ch in str(a["name"]) if ch.isdigit()) or "0"),
    )
    static_ids = []
    ped_ids = []
    header = ["timestamp"]
    row = ["0.0"]
    for a in actors_sorted:
        name = str(a["name"])
        digits = "".join(ch for ch in name if ch.isdigit())
        if not digits:
            continue
        aid = int(digits)
        role = str(a.get("role", "")).lower()
        if role == "pedestrian":
            ped_ids.append(aid)
        else:
            static_ids.append(aid)
        x = float(a.get("x", 0.0))
        y = float(a.get("y", 0.0))
        yaw = float(a.get("yaw", 0.0))
        qx, qy, qz, qw = _yaw_quat(yaw)
        header += [
            f"actor{aid}_x",
            f"actor{aid}_y",
            f"actor{aid}_qx",
            f"actor{aid}_qy",
            f"actor{aid}_qz",
            f"actor{aid}_qw",
        ]
        row += [f"{x}", f"{y}", f"{qx}", f"{qy}", f"{qz}", f"{qw}"]
    (csv_dir / "static_actors_snapshot.csv").write_text(
        ",".join(header) + "\n" + ",".join(row) + "\n", encoding="utf-8"
    )

    exp_name = args.exp_name or csv_dir.name
    main_csv = f"{exp_name}_extracted_optimized.csv"
    # Prefer existing *extracted_optimized.csv
    cands = sorted(csv_dir.glob("*_extracted_optimized.csv"))
    if cands:
        main_csv = cands[0].name

    # 5) Append / ensure data_config.m matlab pack block
    dc = csv_dir / "data_config.m"
    block = f"""
% --- matlab scenario pack (Track D GT) ---
% Người/nhóm/inter/emotion: social_scenario.yaml (canonical)
% Quỹ đạo robot: CSV từ bag (/amcl_pose)
main_wall_file = fullfile(data_path, 'wall_segments.txt');
main_objects_file = fullfile(data_path, 'environment_objects.txt');
main_social_yaml_file = fullfile(data_path, 'social_scenario.yaml');
"""
    if dc.is_file():
        text = dc.read_text(encoding="utf-8")
        if "main_social_yaml_file" not in text:
            dc.write_text(text.rstrip() + "\n" + block, encoding="utf-8")
    else:
        dc.write_text(
            f"% Auto-generated Track D MATLAB pack\ndata_path = '{csv_dir}';\n" + block,
            encoding="utf-8",
        )

    # 6) run_draw_this_exp.m
    note = {
        "real_tn1": "proxemic_only",
        "real_tn2": "social_layers",
        "real_tn3": "density_gate",
        "real_tn4": "proxemic_only_pos",
        "real_tn5": "social_layers_pos",
        "real_tn6": "density_gate_pos",
    }.get(args.exp_variant, args.exp_variant or "track_d")

    draw = f"""% Auto-generated — Track D MATLAB pack (SII/SGI ground-truth scenario)
this_dir = fileparts(mfilename('fullpath'));
if isempty(this_dir), this_dir = pwd; end
if ~exist('EXP_DIR', 'var') || isempty(EXP_DIR), EXP_DIR = this_dir; end

main_csv_file = fullfile(EXP_DIR, '{main_csv}');
main_wall_file = fullfile(EXP_DIR, 'wall_segments.txt');
main_objects_file = fullfile(EXP_DIR, 'environment_objects.txt');
main_social_yaml_file = fullfile(EXP_DIR, 'social_scenario.yaml');

pedestrian_actor_ids = [{' '.join(str(i) for i in ped_ids)}];
static_actor_ids     = [{' '.join(str(i) for i in static_ids)}];

emotion_space_fill      = true;
group_space_fill        = true;
interaction_space_fill  = true;
proxemic_space_fill     = true;

assignin('base', 'main_csv_file', main_csv_file);
assignin('base', 'main_wall_file', main_wall_file);
assignin('base', 'main_objects_file', main_objects_file);
assignin('base', 'main_social_yaml_file', main_social_yaml_file);
assignin('base', 'pedestrian_actor_ids', pedestrian_actor_ids);
assignin('base', 'static_actor_ids', static_actor_ids);
assignin('base', 'EXP_DIR', EXP_DIR);

fprintf('Track D MATLAB pack — {args.exp_variant}\\n');
fprintf('  Note   : {note} | SII/SGI GT from social_scenario.yaml\\n');
fprintf('  Static : %s\\n', mat2str(static_actor_ids));
fprintf('  Walk   : %s\\n', mat2str(pedestrian_actor_ids));
fprintf('  CSV    : %s\\n', main_csv_file);
fprintf('  YAML   : %s\\n', main_social_yaml_file);
fprintf('Tiếp: cd <research-pipeline-os>\\\\matlab\\\\debug_quick_start rồi chạy quick_start\\n');
"""
    (csv_dir / "run_draw_this_exp.m").write_text(draw, encoding="utf-8")

    manifest = [
        f"exp_variant={args.exp_variant}",
        f"main_csv={main_csv}",
        f"actors={len(actors_sorted)}",
        f"static_ids={static_ids}",
        f"pedestrian_ids={ped_ids}",
        f"objects={len(data.get('objects') or [])}",
        f"groups={len(data.get('groups') or [])}",
        f"interactions={len(data.get('interactions') or [])}",
        f"canonical={canonical}",
        f"matlab_yaml={matlab_yaml}",
        "sii_sgi_basis=scenario_gt_not_sensor",
    ]
    (csv_dir / "matlab_pack_manifest.txt").write_text("\n".join(manifest) + "\n", encoding="utf-8")
    print(f"[matlab-pack] OK → {csv_dir}")
    print(f"  social_scenario.yaml (GT) + objects + actors snapshot")


if __name__ == "__main__":
    main()
