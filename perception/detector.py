from ultralytics import YOLO

from perception.object_policy import (
    PERSON_LABEL,
    PERSON_MIN_CONFIDENCE,
)


class SafetyDetector:

    def __init__(
        self,
        model_path="yolo26n.pt",
        confidence=0.25
    ):

        print(
            f"[AI] Loading model: "
            f"{model_path}"
        )

        self.model = YOLO(
            model_path
        )

        # This is now an inference FLOOR,
        # not the final navigation threshold.
        self.confidence = confidence

        self.person_class_id = None

        for class_id, name in (
            self.model.names.items()
        ):

            if (
                name.lower()
                == PERSON_LABEL
            ):

                self.person_class_id = int(
                    class_id
                )

                break

        if self.person_class_id is None:

            raise RuntimeError(
                "Model does not contain "
                "a person class"
            )

        print(
            "[AI] Person class ID:",
            self.person_class_id
        )

    # ==================================================

    def detect_scene(
        self,
        frame
    ):

        results = self.model.predict(
            source=frame,
            conf=self.confidence,
            imgsz=320,
            verbose=False
        )

        result = results[0]

        detections = []

        if result.boxes is None:

            return (
                detections,
                result
            )

        for box in result.boxes:

            class_id = int(
                box.cls[0].cpu()
            )

            confidence = float(
                box.conf[0].cpu()
            )

            x1, y1, x2, y2 = [
                int(value)
                for value
                in (
                    box.xyxy[0]
                    .cpu()
                    .tolist()
                )
            ]

            label = (
                self.model.names[
                    class_id
                ].lower()
            )

            detections.append({
                "class_id":
                    class_id,

                "label":
                    label,

                "confidence":
                    confidence,

                "bbox": (
                    x1,
                    y1,
                    x2,
                    y2
                )
            })

        return (
            detections,
            result
        )

    # ==================================================

    @staticmethod
    def get_people(
        detections
    ):

        return [
            detection
            for detection in detections

            if (
                detection["label"]
                == PERSON_LABEL

                and

                detection["confidence"]
                >= PERSON_MIN_CONFIDENCE
            )
        ]

    # Keep compatibility with your older test.
    def detect_people(
        self,
        frame
    ):

        detections, result = (
            self.detect_scene(
                frame
            )
        )

        return (
            self.get_people(
                detections
            ),
            result
        )