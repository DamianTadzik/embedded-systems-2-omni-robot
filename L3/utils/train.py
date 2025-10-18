from ultralytics import YOLO

yolo = YOLO("yolov8n.pt")
trained_yolo = yolo.train(data="D:\Studia\embeded2\monster_db\data.yaml", epochs=3, imgsz=640)