#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Xuất layout annotation (RViz) → canonical scenario + MATLAB yaml (+ sync runtime).

CLI:
  rosrun social_density_fusion export_annotation_to_canonical.py \\
    --source $(rospack find social_density_fusion)/config/D-density-costmap/real/social_actor_layout_from_sim.yaml

  # kèm pack MATLAB vào thư mục CSV:
  ... --matlab-pack-dir ~/omni4wd_ws/csv/D-density-costmap/annotation_preview
"""
from __future__ import annotations

import argparse
import copy
import subprocess
import sys
from datetime import date
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

CFG_REAL = (
    Path(__file__).resolve().parents[1]
    / "config"
    / "D-density-costmap"
    / "real"
)


def _load(path: Path) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _dump(path: Path, data: Dict[str, Any], header: str = "") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    body = yaml.safe_dump(
        data,
        default_flow_style=False,
        allow_unicode=True,
        sort_keys=False,
    )
    with open(path, "w", encoding="utf-8") as f:
        if header:
            f.write(header if header.endswith("\n") else header + "\n")
        f.write(body)


def _f(v: Any, nd: int = 2) -> float:
    return round(float(v), nd)


def _actor_id(name: str) -> int:
    digits = "".join(ch for ch in str(name) if ch.isdigit())
    return int(digits) if digits else 0


def _centroid(members: List[Dict[str, Any]]) -> List[float]:
    if not members:
        return [0.0, 0.0]
    sx = sum(float(a.get("x", 0.0)) for a in members)
    sy = sum(float(a.get("y", 0.0)) for a in members)
    n = float(len(members))
    return [_f(sx / n, 3), _f(sy / n, 3)]


def _build_groups(
    actors: List[Dict[str, Any]],
    source_groups: Optional[List[Dict[str, Any]]],
    fallback_groups: Optional[List[Dict[str, Any]]],
) -> List[Dict[str, Any]]:
    actors_by_name = {
        str(a["name"]): a for a in actors if isinstance(a, dict) and "name" in a
    }
    template = source_groups or fallback_groups or []

    if template:
        out = []
        for g in template:
            if not isinstance(g, dict):
                continue
            gname = str(g.get("group_name") or g.get("name") or "")
            members = [str(m) for m in (g.get("members") or [])]
            poses = [actors_by_name[m] for m in members if m in actors_by_name]
            out.append(
                {
                    "group_name": gname,
                    "size": len(members),
                    "members": members,
                    "center_xy": _centroid(poses),
                }
            )
        return out

    # Gom theo field actor.group
    by_group: Dict[str, List[Dict[str, Any]]] = {}
    for a in actors:
        if not isinstance(a, dict) or "name" not in a:
            continue
        if str(a.get("role", "")).lower() == "pedestrian":
            continue
        g = str(a.get("group") or "").strip()
        if not g:
            continue
        by_group.setdefault(g, []).append(a)

    out = []
    for gname in sorted(by_group.keys()):
        members = sorted(by_group[gname], key=lambda a: _actor_id(a["name"]))
        out.append(
            {
                "group_name": gname,
                "size": len(members),
                "members": [str(m["name"]) for m in members],
                "center_xy": _centroid(members),
            }
        )
    return out


def _build_interactions(
    source: Dict[str, Any],
    fallback: Optional[List[Dict[str, Any]]] = None,
) -> List[Dict[str, Any]]:
    inter = source.get("interactions")
    if inter:
        return copy.deepcopy(inter)
    if fallback:
        return copy.deepcopy(fallback)
    return []


def _build_people(actors: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    people = []
    for a in sorted(
        [x for x in actors if isinstance(x, dict) and "name" in x],
        key=lambda x: _actor_id(x["name"]),
    ):
        people.append(
            {
                "name": str(a["name"]),
                "emotion": str(a.get("emotion") or "Neutral"),
            }
        )
    return people


def build_canonical(
    source: Dict[str, Any],
    base_canonical: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Ghép actors/objects/zones từ annotation vào khung canonical."""
    base = copy.deepcopy(base_canonical) if base_canonical else {}
    actors = copy.deepcopy(source.get("actors") or base.get("actors") or [])
    objects = copy.deepcopy(source.get("objects") or base.get("objects") or [])
    zones = copy.deepcopy(source.get("emotion_zones") or base.get("emotion_zones") or [])

    for a in actors:
        if not isinstance(a, dict):
            continue
        if "x" in a:
            a["x"] = _f(a["x"], 2)
        if "y" in a:
            a["y"] = _f(a["y"], 2)
        if "yaw" in a:
            a["yaw"] = _f(a["yaw"], 3)
        # Pedestrian path: waypoints + walk_goal (điểm cuối)
        if str(a.get("role", "")).lower() == "pedestrian":
            wps = a.get("waypoints")
            clean = []
            if isinstance(wps, list):
                for p in wps:
                    if isinstance(p, dict) and "x" in p and "y" in p:
                        clean.append({"x": _f(p["x"], 2), "y": _f(p["y"], 2)})
            wg = a.get("walk_goal")
            if not clean and isinstance(wg, dict) and "x" in wg and "y" in wg:
                clean = [{"x": _f(wg["x"], 2), "y": _f(wg["y"], 2)}]
            if clean:
                a["waypoints"] = clean
                a["walk_goal"] = {"x": clean[-1]["x"], "y": clean[-1]["y"]}
            elif isinstance(wg, dict) and "x" in wg and "y" in wg:
                a["walk_goal"] = {"x": _f(wg["x"], 2), "y": _f(wg["y"], 2)}
    for o in objects:
        if not isinstance(o, dict):
            continue
        for k in ("x", "y", "z"):
            if k in o:
                o[k] = _f(o[k], 2)
    for z in zones:
        if not isinstance(z, dict):
            continue
        for k in ("x_min", "y_min", "x_max", "y_max"):
            if k in z:
                z[k] = _f(z[k], 2)

    groups = _build_groups(actors, source.get("groups"), base.get("groups"))
    interactions = _build_interactions(source, base.get("interactions"))

    meta = copy.deepcopy(base.get("meta") or {})
    meta.setdefault("track", "D-density-costmap")
    meta.setdefault("scenario_id", "trackD_real_canonical_v1")
    meta.setdefault("frame_id", "map")
    meta["date"] = str(date.today())
    meta["notes"] = (
        "Xuất từ social_map_annotation_editor (RViz). "
        "SII/SGI GT + MATLAB dùng file này."
    )

    density = copy.deepcopy(
        source.get("density") or base.get("density") or {"button_state": False}
    )

    out: Dict[str, Any] = {
        "meta": meta,
        "density": density,
        "people": _build_people(actors),
        "groups": groups,
        "interactions": interactions,
        "actors": actors,
        "emotion_zones": zones,
        "objects": objects,
    }
    if base.get("robot_waypoints"):
        out["robot_waypoints"] = copy.deepcopy(base["robot_waypoints"])
    elif source.get("robot_waypoints"):
        out["robot_waypoints"] = copy.deepcopy(source["robot_waypoints"])
    return out


def build_matlab_yaml(canonical: Dict[str, Any]) -> Dict[str, Any]:
    """Bản rút gọn cho parse_social_yaml_config.m."""
    angry = [
        {"name": p["name"], "emotion": p["emotion"]}
        for p in (canonical.get("people") or [])
        if isinstance(p, dict) and str(p.get("emotion", "")).lower() == "angry"
    ]
    groups = []
    for g in canonical.get("groups") or []:
        if not isinstance(g, dict):
            continue
        groups.append(
            {
                "group_name": g.get("group_name"),
                "members": list(g.get("members") or []),
            }
        )
    pedestrians = []
    for a in canonical.get("actors") or []:
        if not isinstance(a, dict):
            continue
        if str(a.get("role", "")).lower() != "pedestrian":
            continue
        wps = []
        for p in a.get("waypoints") or []:
            if isinstance(p, dict) and "x" in p and "y" in p:
                wps.append({"x": _f(p["x"], 2), "y": _f(p["y"], 2)})
        wg = a.get("walk_goal") or (wps[-1] if wps else {})
        entry = {
            "name": str(a.get("name")),
            "start": {"x": _f(a.get("x", 0.0), 2), "y": _f(a.get("y", 0.0), 2)},
            "waypoints": wps,
        }
        if isinstance(wg, dict) and "x" in wg and "y" in wg:
            entry["walk_goal"] = {"x": _f(wg["x"], 2), "y": _f(wg["y"], 2)}
        pedestrians.append(entry)
    out = {
        "density": copy.deepcopy(canonical.get("density") or {"button_state": False}),
        "people": angry,
        "groups": groups,
        "interactions": copy.deepcopy(canonical.get("interactions") or []),
    }
    if pedestrians:
        out["pedestrians"] = pedestrians
    return out


def sync_runtime_yamls(
    canonical: Dict[str, Any],
    actor_layout_path: Optional[Path] = None,
    exp4_path: Optional[Path] = None,
    exp5_path: Optional[Path] = None,
    exp6_path: Optional[Path] = None,
) -> List[str]:
    """Đồng bộ layout + social_config exp4/5/6 từ canonical."""
    written: List[str] = []
    actors = copy.deepcopy(canonical.get("actors") or [])
    objects = copy.deepcopy(canonical.get("objects") or [])
    zones = copy.deepcopy(canonical.get("emotion_zones") or [])
    groups_slim = [
        {"group_name": g.get("group_name"), "members": list(g.get("members") or [])}
        for g in (canonical.get("groups") or [])
        if isinstance(g, dict)
    ]
    interactions = copy.deepcopy(canonical.get("interactions") or [])
    people = copy.deepcopy(canonical.get("people") or [])
    ped_names = {
        str(a.get("name"))
        for a in actors
        if isinstance(a, dict) and str(a.get("role", "")).lower() == "pedestrian"
    }
    people_static = [
        p for p in people if isinstance(p, dict) and str(p.get("name")) not in ped_names
    ]

    if actor_layout_path:
        layout = {
            "actors": actors,
            "emotion_zones": zones,
            "objects": objects,
            "groups": groups_slim,
            "interactions": interactions,
        }
        header = (
            "# Track D — layout actor (đồng bộ social_scenario_canonical_trackD_real.yaml)\n"
            "# Auto-exported by export_annotation_to_canonical.\n"
        )
        _dump(actor_layout_path, layout, header)
        written.append(str(actor_layout_path))

    if exp4_path:
        data = _load(exp4_path) if exp4_path.is_file() else {}
        data["density"] = dict(data.get("density") or {})
        data["density"]["button_state"] = False
        det = dict(data.get("detection") or {})
        det["enable_group_detection"] = False
        det["enable_interaction_detection"] = False
        data["detection"] = det
        data["people"] = people_static
        data["emotion_zones"] = []
        data["objects"] = []
        data["groups"] = []
        data["interactions"] = []
        header = (
            "# Auto-synced from canonical by export_annotation_to_canonical.\n"
            "# exp4 proxemic-only: people only (no zones/objects/groups).\n"
        )
        _dump(exp4_path, data, header)
        written.append(str(exp4_path))

    for path, button in ((exp5_path, False), (exp6_path, True)):
        if not path:
            continue
        try:
            data = _load(path) if path.is_file() else {}
        except Exception:
            data = {}
        data["density"] = dict(data.get("density") or {})
        data["density"]["button_state"] = button
        data["people"] = people_static
        data["actors"] = actors
        data["emotion_zones"] = []
        for z in zones:
            if not isinstance(z, dict):
                continue
            data["emotion_zones"].append({k: z[k] for k in z if k != "ref_actor"})
        data["objects"] = objects
        data["groups"] = groups_slim
        data["interactions"] = interactions
        header = (
            "# Auto-synced from canonical by export_annotation_to_canonical.\n"
            "# objects + emotion_zones + people/groups/interactions + actors.\n"
        )
        _dump(path, data, header)
        written.append(str(path))
    return written


def export_scenario(
    source: Dict[str, Any],
    canonical_path: Path,
    matlab_path: Path,
    base_canonical_path: Optional[Path] = None,
    sync_layout: bool = True,
    sync_exp_configs: bool = True,
    actor_layout_path: Optional[Path] = None,
    exp4_path: Optional[Path] = None,
    exp5_path: Optional[Path] = None,
    exp6_path: Optional[Path] = None,
    matlab_pack_dir: Optional[Path] = None,
    exp_variant: str = "",
) -> Tuple[Dict[str, Any], List[str]]:
    if base_canonical_path and base_canonical_path.is_file():
        base = _load(base_canonical_path)
    elif canonical_path.is_file():
        base = _load(canonical_path)
    else:
        base = {}

    canonical = build_canonical(source, base)

    canon_header = (
        "# =============================================================================\n"
        "# Track D — KỊCH BẢN CHUẨN (canonical) — xuất từ RViz annotation\n"
        "# =============================================================================\n"
        "# Nguồn: social_map_annotation_editor → export_annotation_to_canonical\n"
        "# Frame: map | date: %s\n"
        "# =============================================================================\n"
        % date.today()
    )
    _dump(canonical_path, canonical, canon_header)

    matlab = build_matlab_yaml(canonical)
    matlab_header = (
        "# Auto-generated MATLAB scenario — Track D real CANONICAL\n"
        "# Nguồn: social_scenario_canonical_trackD_real.yaml\n"
        "# Dùng cho research-pipeline-os/matlab/... (parse_social_yaml_config.m)\n"
    )
    _dump(matlab_path, matlab, matlab_header)

    written = [str(canonical_path), str(matlab_path)]

    layout = actor_layout_path or (CFG_REAL / "social_actor_layout_from_sim.yaml")
    e4 = exp4_path or (CFG_REAL / "social_config_real_exp4_proxemic_only_pos.yaml")
    e5 = exp5_path or (CFG_REAL / "social_config_real_exp5_social_layers_pos.yaml")
    e6 = exp6_path or (CFG_REAL / "social_config_real_exp6_density_gate_pos.yaml")
    written += sync_runtime_yamls(
        canonical,
        actor_layout_path=layout if sync_layout else None,
        exp4_path=e4 if sync_exp_configs else None,
        exp5_path=e5 if sync_exp_configs else None,
        exp6_path=e6 if sync_exp_configs else None,
    )

    if matlab_pack_dir:
        pack_script = (
            Path(__file__).resolve().parents[3] / "scripts" / "export_track_d_matlab_pack.py"
        )
        if not pack_script.is_file():
            pack_script = Path.home() / "omni4wd_ws" / "scripts" / "export_track_d_matlab_pack.py"
        matlab_pack_dir.mkdir(parents=True, exist_ok=True)
        cmd = [
            sys.executable,
            str(pack_script),
            str(matlab_pack_dir),
            "--canonical",
            str(canonical_path),
            "--matlab-yaml",
            str(matlab_path),
            "--exp-variant",
            exp_variant or "annotation",
            "--exp-name",
            matlab_pack_dir.name,
        ]
        subprocess.check_call(cmd)
        written.append(str(matlab_pack_dir))

    return canonical, written


def main() -> None:
    ap = argparse.ArgumentParser(description="Annotation → canonical + MATLAB")
    ap.add_argument(
        "--source",
        type=Path,
        default=CFG_REAL / "social_actor_layout_from_sim.yaml",
        help="YAML annotation (actors/objects/zones[/groups/interactions])",
    )
    ap.add_argument(
        "--canonical",
        type=Path,
        default=CFG_REAL / "social_scenario_canonical_trackD_real.yaml",
    )
    ap.add_argument(
        "--matlab",
        type=Path,
        default=CFG_REAL / "social_scenario_matlab_trackD_real.yaml",
    )
    ap.add_argument("--no-sync-layout", action="store_true")
    ap.add_argument("--no-sync-exp", action="store_true")
    ap.add_argument("--matlab-pack-dir", type=Path, default=None)
    ap.add_argument("--exp-variant", default="")
    args = ap.parse_args()

    if not args.source.is_file():
        raise SystemExit(f"Missing source: {args.source}")

    source = _load(args.source)
    _, written = export_scenario(
        source,
        canonical_path=args.canonical,
        matlab_path=args.matlab,
        base_canonical_path=args.canonical,
        sync_layout=not args.no_sync_layout,
        sync_exp_configs=not args.no_sync_exp,
        matlab_pack_dir=args.matlab_pack_dir,
        exp_variant=args.exp_variant,
    )
    print("[export-canonical] OK")
    for w in written:
        print(f"  → {w}")


if __name__ == "__main__":
    main()
