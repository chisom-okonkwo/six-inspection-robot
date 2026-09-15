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

        # Find the model's class ID for "person"
        self.person_class_id = None

        for class_id, name in (
            self.model.names.items()
        ):

            if name.lower() == "person":

                self.person_class_id = (
                    int(class_id)
                )

                break

        if self.person_class_id is None:

            raise RuntimeError(
                "This model does not contain "
                "a 'person' class"
            )

        print(
            "[AI] Person class ID:",
            self.person_class_id
        )

    # --------------------------------------------------

    def detect_people(self, frame):

        results = self.model.predict(
            source=frame,
            conf=self.confidence,
            classes=[
                self.person_class_id
            ],
            imgsz=320,
            verbose=False
        )

        result = results[0]

        detections = []

        if result.boxes is not None:

            for box in result.boxes:

                coordinates = (
                    box.xyxy[0]
                    .cpu()
                    .tolist()
                )

                confidence = float(
                    box.conf[0]
                    .cpu()
                )

                x1, y1, x2, y2 = [
                    int(value)
                    for value in coordinates
                ]

                detections.append({
                    "label": "person",
                    "confidence": confidence,
                    "bbox": (
                        x1,
                        y1,
                        x2,
                        y2
                    )
                })

        return detections, result
