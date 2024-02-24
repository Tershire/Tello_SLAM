from ultralytics import YOLO

# load
model = YOLO("./runs/detect/train5/weights/best.pt")
# model = YOLO("./models/yolov8n.pt")

# export
model.export(format="onnx", opset=12, simplify=True, dynamic=False, imgsz=640)
# model.export(format="onnx", opset=12, imgsz=640)
