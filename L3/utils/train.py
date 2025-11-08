from ultralytics import YOLO
import os

yolo = YOLO("yolov8n.pt")
data_path = os.getcwd() + "\\monster_db\\"

if not os.path.exists(data_path):
    raise FileExistsError(f"Path {data_path} does not exist!")

trained_yolo = yolo.train(data=data_path+"data.yaml", epochs=10, imgsz=640, cfg=data_path+"cfg.yaml")