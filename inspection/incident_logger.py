from datetime import datetime
from pathlib import Path

import json
import threading

import cv2


class IncidentLogger:

    def __init__(
        self,
        project_root=None
    ):

        if project_root is None:

            project_root = (
                Path(__file__)
                .resolve()
                .parent
                .parent
            )

        self.project_root = Path(
            project_root
        )

        self.logs_dir = (
            self.project_root
            / "data"
            / "logs"
        )

        self.snapshots_dir = (
            self.project_root
            / "data"
            / "snapshots"
        )

        self.logs_dir.mkdir(
            parents=True,
            exist_ok=True
        )

        self.snapshots_dir.mkdir(
            parents=True,
            exist_ok=True
        )

        # One identifier for the entire
        # inspection run.
        self.session_id = (
            datetime.now()
            .astimezone()
            .strftime(
                "%Y%m%d_%H%M%S"
            )
        )

        self.log_path = (
            self.logs_dir
            / (
                "inspection_"
                f"{self.session_id}.jsonl"
            )
        )

        self._lock = (
            threading.Lock()
        )

        print(
            "[INSPECTION] Log file:"
        )

        print(
            f"[INSPECTION] "
            f"{self.log_path}"
        )

    # ==================================================
    # LOG INCIDENT
    # ==================================================

    def log_incident(
        self,
        event_type,
        state,
        frame=None,
        details=None
    ):

        now = (
            datetime.now()
            .astimezone()
        )

        timestamp = (
            now.isoformat(
                timespec="milliseconds"
            )
        )

        snapshot_path = None

        # ==============================================
        # SAVE VISUAL EVIDENCE
        # ==============================================

        if frame is not None:

            snapshot_name = (
                f"{self.session_id}_"
                f"{now.strftime('%H%M%S_%f')}_"
                f"{event_type}.jpg"
            )

            full_snapshot_path = (
                self.snapshots_dir
                / snapshot_name
            )

            saved = cv2.imwrite(
                str(full_snapshot_path),
                frame
            )

            if saved:

                # Store relative path in log so the
                # project remains portable.
                snapshot_path = str(
                    full_snapshot_path.relative_to(
                        self.project_root
                    )
                )

        # ==============================================
        # BUILD RECORD
        # ==============================================

        record = {

            "timestamp":
                timestamp,

            "session_id":
                self.session_id,

            "event_type":
                event_type,

            # ------------------------------
            # Person detection
            # ------------------------------

            "person_detected":
                state.get(
                    "person_detected",
                    False
                ),

            "person_confidence":
                state.get(
                    "person_confidence",
                    0.0
                ),

            # ------------------------------
            # Obstacle detection
            # ------------------------------

            "obstacle_detected":
                state.get(
                    "obstacle_detected",
                    False
                ),

            "obstacle_label":
                state.get(
                    "obstacle_label"
                ),

            "obstacle_confidence":
                state.get(
                    "obstacle_confidence",
                    0.0
                ),

            "obstacle_side":
                state.get(
                    "obstacle_side"
                ),

            # ------------------------------
            # Robot state
            # ------------------------------

            "safety_state":
                state.get(
                    "safety_state"
                ),

            "motion_state":
                state.get(
                    "motion_state"
                ),

            "system_state":
                state.get(
                    "system_state"
                ),

            # ------------------------------
            # Performance
            # ------------------------------

            "perception_fps":
                state.get(
                    "perception_fps",
                    0.0
                ),

            # ------------------------------
            # Evidence
            # ------------------------------

            "snapshot":
                snapshot_path
        }

        if details is not None:

            record["details"] = (
                details
            )

        # ==============================================
        # WRITE JSONL
        # ==============================================

        with self._lock:

            with open(
                self.log_path,
                "a",
                encoding="utf-8"
            ) as file:

                file.write(
                    json.dumps(
                        record
                    )
                )

                file.write(
                    "\n"
                )

        print(
            "[INSPECTION] "
            f"Recorded: {event_type}"
        )

        return record