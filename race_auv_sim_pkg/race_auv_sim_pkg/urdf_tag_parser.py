"""Parse a URDF to extract the pose of every AprilTag link in the object's base frame.

Convention: a tag with id ``N`` is a child link named ``f"{prefix}{N}"``
rigidly fixed-jointed to the object's base link. The pose we want is the
homogeneous transform from the base frame to the tag frame.

We use a tiny stdlib-only XML walker instead of pulling in
``urdf_parser_py``; the only fields we need are ``<joint>``'s ``child``
link name and its optional ``<origin xyz rpy/>`` child.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Set
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation as R


def _parse_floats(text: Optional[str], n: int) -> list[float]:
    if not text:
        return [0.0] * n
    parts = text.replace(",", " ").split()
    vals = [float(x) for x in parts]
    if len(vals) < n:
        vals += [0.0] * (n - len(vals))
    return vals[:n]


def _origin_to_homogeneous(origin: Optional[ET.Element]) -> np.ndarray:
    xyz = _parse_floats(origin.get("xyz") if origin is not None else None, 3)
    rpy = _parse_floats(origin.get("rpy") if origin is not None else None, 3)
    T = np.eye(4)
    T[:3, :3] = R.from_euler('xyz', rpy).as_matrix()
    T[:3, 3] = xyz
    return T


@dataclass
class TagTransform:
    tag_id: int
    child_link: str
    T_base_to_tag: np.ndarray  # 4x4


def _load_link_names(urdf_path: Path) -> Set[str]:
    root = ET.parse(str(urdf_path)).getroot()
    return {lnk.get("name") for lnk in root.findall("link") if lnk.get("name")}


def _resolve_base_link(urdf_path: Path, base_link_name: Optional[str]) -> str:
    link_names = _load_link_names(urdf_path)
    if base_link_name:
        if base_link_name not in link_names:
            raise ValueError(
                f"base_link_name '{base_link_name}' not in URDF link set"
            )
        return base_link_name
    root = ET.parse(str(urdf_path)).getroot()
    child_links = set()
    for j in root.findall("joint"):
        ch = j.find("child")
        if ch is not None and ch.get("link"):
            child_links.add(ch.get("link"))
    roots = [lnk.get("name") for lnk in root.findall("link") if lnk.get("name")]
    for name in roots:
        if name not in child_links:
            return name
    if roots:
        return roots[0]
    raise ValueError(f"No <link> elements found in {urdf_path}")


def extract_tag_transforms(
    urdf_path: str | Path,
    prefix: str = "apriltag",
    base_link_name: Optional[str] = None,
) -> Dict[int, TagTransform]:
    """Walk the URDF joints and return the base->tag transform for every tag.

    Parameters
    ----------
    urdf_path
        Path to the URDF XML file.
    prefix
        Link-name prefix used for tags (default ``"apriltag"``). A child link
        named ``"{prefix}{int}"`` is treated as a tag.
    base_link_name
        Name of the object's base link. If ``None``, the URDF root link is
        used. Tags fixed-jointed to a sub-tree that does not descend from the
        base link are silently skipped.

    Returns
    -------
    dict
        ``{tag_id: TagTransform}`` for every matching joint.
    """
    urdf_path = Path(urdf_path).expanduser().resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    base = _resolve_base_link(urdf_path, base_link_name)
    link_names = _load_link_names(urdf_path)
    if base not in link_names:
        raise ValueError(f"Resolved base link '{base}' missing from URDF")

    root = ET.parse(str(urdf_path)).getroot()
    result: Dict[int, TagTransform] = {}
    for joint in root.findall("joint"):
        if joint.get("type", "").lower() != "fixed":
            continue
        child_el = joint.find("child")
        if child_el is None:
            continue
        child = child_el.get("link", "")
        if not child.startswith(prefix):
            continue
        suffix = child[len(prefix):]
        if not suffix.isdigit():
            continue
        tag_id = int(suffix)
        T = _origin_to_homogeneous(joint.find("origin"))
        result[tag_id] = TagTransform(
            tag_id=tag_id, child_link=child, T_base_to_tag=T
        )

    if not result:
        raise ValueError(
            f"No '{prefix}<int>' fixed-jointed links found in {urdf_path}"
        )
    return result
