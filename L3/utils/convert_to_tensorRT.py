from ultralytics import YOLO

model = YOLO("monster_net_0.2.pt")

# Export the model to TensorRT
model.export(format="engine")

# DLA enable version
# model.export(format="engine", device="dla:0", half=True)  # dla:0 or dla:1 corresponds to the DLA cores
