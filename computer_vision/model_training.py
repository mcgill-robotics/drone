from ultralytics import YOLO
import os
if __name__ == "__main__":
    # load model
    model = YOLO("yolov8n.yaml")  # build new model from scratch

    root_dir = os.path.dirname(__file__)
    # use the model
    results = model.train(
        data=os.path.join(root_dir, "dataset", "data.yaml"),
        epochs=10)

    results = model(os.path.join(root_dir, "dataset",
                    "test", "images", "00001.png"))
    print(results)
