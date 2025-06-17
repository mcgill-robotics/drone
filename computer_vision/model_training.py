from ultralytics import YOLO
import os
if __name__ == "__main__":
    # load model

    root_dir = os.path.dirname(__file__)
    model = YOLO(
        os.path.join(root_dir, "..", "runs", "detect", "train4", "weights",
                     "last.pt"))  # build new model from scratch
    # use the model
    results = model.train(data=os.path.join(root_dir, "dataset", "data.yaml"),
                          epochs=1)

    results = model(
        os.path.join(root_dir, "dataset", "test", "images", "00001.png"))
    print(results)
