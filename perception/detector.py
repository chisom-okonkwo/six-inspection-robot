from ultralytics import YOLO


class SafetyDetector:

    def __init__(
        self,
        model_path="yolo26n.pt",
        confidence=0.45
    ):

        print(
            f"[AI] Loading model: "
            f"{model_path}"
        )

        self.model = YOLO(
            model_path
        )

        self.confidence = confidence

        self.person_class_id = None

        for class_id, name in (
            self.model.names.items()
        ):

            if name.lower() == "person":

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
    # FULL SCENE DETECTION
    # ==================================================

    def detect_scene(self, frame):

        results = self.model.predict(
            source=frame,
            conf=self.confidence,
            imgsz=320,
            verbose=False
        )

        result = results[0]

        detections = []

        if result.boxes is None:

            return detections, result

        for box in result.boxes:

            class_id = int(
                box.cls[0].cpu()
            )

            confidence = float(
                box.conf[0].cpu()
            )

            coordinates = (
                box.xyxy[0]
                .cpu()
                .tolist()
            )

            x1, y1, x2, y2 = [
                int(value)
                for value in coordinates
            ]

            label = self.model.names[
                class_id
            ]

            detections.append({
                "class_id": class_id,
                "label": label.lower(),
                "confidence": confidence,
                "bbox": (
                    x1,
                    y1,
                    x2,
                    y2
                )
            })

        return detections, result

    # ==================================================
    # FILTER PEOPLE
    # ==================================================

    @staticmethod
    def get_people(detections):

        return [
            detection
            for detection in detections
            if detection["label"] == "person"
        ]

    # ==================================================
    # BACKWARDS COMPATIBILITY
    # ==================================================

    def detect_people(self, frame):

        detections, result = (
            self.detect_scene(frame)
        )

        people = self.get_people(
            detections
        )

        return people, result