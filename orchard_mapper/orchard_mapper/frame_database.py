"""SQLite metadata plus PNG frame storage for loop-closure re-rendering."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import sqlite3
import threading

import cv2
import numpy as np

from .coordinate_transform import Pose2D


@dataclass(frozen=True)
class FrameRecord:
    frame_id: int
    stamp_ns: int
    image_path: str
    mask_path: str
    map_pose: Pose2D
    odom_pose: Pose2D | None


def _atomic_png(path: Path, image: np.ndarray) -> None:
    ok, encoded = cv2.imencode(".png", image)
    if not ok:
        raise OSError(f"failed to encode {path}")
    temporary = path.with_suffix(path.suffix + ".tmp")
    with open(temporary, "wb") as output:
        output.write(encoded.tobytes())
        output.flush()
        os.fsync(output.fileno())
    os.replace(temporary, path)


class FrameDatabase:
    def __init__(self, root: str) -> None:
        self.root = Path(root)
        self.frames_dir = self.root / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.connection = sqlite3.connect(
            self.root / "frames.sqlite3", check_same_thread=False
        )
        self.lock = threading.RLock()
        self.connection.execute("PRAGMA journal_mode=WAL")
        self.connection.execute(
            """
            CREATE TABLE IF NOT EXISTS frames (
              id INTEGER PRIMARY KEY AUTOINCREMENT,
              stamp_ns INTEGER NOT NULL,
              image_path TEXT NOT NULL,
              mask_path TEXT NOT NULL,
              map_x REAL NOT NULL,
              map_y REAL NOT NULL,
              map_yaw REAL NOT NULL,
              odom_x REAL,
              odom_y REAL,
              odom_yaw REAL
            )
            """
        )
        self.connection.execute(
            "CREATE INDEX IF NOT EXISTS frames_stamp ON frames(stamp_ns)"
        )
        self.connection.commit()

    def add(
        self,
        stamp_ns: int,
        image: np.ndarray,
        mask: np.ndarray,
        map_pose: Pose2D,
        odom_pose: Pose2D | None,
    ) -> int:
        with self.lock:
            cursor = self.connection.execute(
                """
                INSERT INTO frames(
                  stamp_ns,image_path,mask_path,map_x,map_y,map_yaw,
                  odom_x,odom_y,odom_yaw
                ) VALUES(?,?,?,?,?,?,?,?,?)
                """,
                (
                    int(stamp_ns),
                    "pending",
                    "pending",
                    map_pose.x,
                    map_pose.y,
                    map_pose.yaw,
                    None if odom_pose is None else odom_pose.x,
                    None if odom_pose is None else odom_pose.y,
                    None if odom_pose is None else odom_pose.yaw,
                ),
            )
            frame_id = int(cursor.lastrowid)
            image_path = self.frames_dir / f"frame_{frame_id:08d}.png"
            mask_path = self.frames_dir / f"mask_{frame_id:08d}.png"
            try:
                _atomic_png(image_path, image)
                _atomic_png(mask_path, mask)
                self.connection.execute(
                    "UPDATE frames SET image_path=?, mask_path=? WHERE id=?",
                    (
                        str(image_path.relative_to(self.root)),
                        str(mask_path.relative_to(self.root)),
                        frame_id,
                    ),
                )
                self.connection.commit()
            except Exception:
                self.connection.rollback()
                self.connection.execute("DELETE FROM frames WHERE id=?", (frame_id,))
                self.connection.commit()
                raise
            return frame_id

    def records(self) -> list[FrameRecord]:
        with self.lock:
            rows = self.connection.execute(
                """
                SELECT id,stamp_ns,image_path,mask_path,map_x,map_y,map_yaw,
                       odom_x,odom_y,odom_yaw
                FROM frames ORDER BY stamp_ns,id
                """
            ).fetchall()
        records: list[FrameRecord] = []
        for row in rows:
            odom_pose = (
                None
                if row[7] is None
                else Pose2D(float(row[7]), float(row[8]), float(row[9]))
            )
            records.append(
                FrameRecord(
                    frame_id=int(row[0]),
                    stamp_ns=int(row[1]),
                    image_path=str(self.root / row[2]),
                    mask_path=str(self.root / row[3]),
                    map_pose=Pose2D(float(row[4]), float(row[5]), float(row[6])),
                    odom_pose=odom_pose,
                )
            )
        return records

    @staticmethod
    def load(record: FrameRecord) -> tuple[np.ndarray, np.ndarray]:
        image = cv2.imread(record.image_path, cv2.IMREAD_COLOR)
        mask = cv2.imread(record.mask_path, cv2.IMREAD_GRAYSCALE)
        if image is None or mask is None:
            raise OSError(f"frame files are unavailable for id={record.frame_id}")
        return image, mask

    def count(self) -> int:
        with self.lock:
            return int(
                self.connection.execute("SELECT COUNT(*) FROM frames").fetchone()[0]
            )

    def close(self) -> None:
        with self.lock:
            self.connection.close()
