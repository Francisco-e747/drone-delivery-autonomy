import cv2
import glob
import os
import time
from ultralytics import YOLO

# ── config
IMAGE_DIR = "../data/visdrone_samples/VisDrone2019-DET-val/images"
OUTPUT_DIR = "../data/visdrone_samples/results"
CONFIDENCE = 0.25
NUM_IMAGES = 10

VISDRONE_CLASSES = {
    0: "pedestrian", 1: "person", 2: "bicycle", 3: "car",
    4: "van", 5: "truck", 6: "tricycle", 7: "awning-tricycle",
    8: "bus", 9: "motorcycle"
}
REMAP = {
    0: "person", 1: "person", 2: "vehicle", 3: "vehicle",
    4: "vehicle", 5: "vehicle", 6: "vehicle", 7: "vehicle",
    8: "vehicle", 9: "vehicle"
}

os.makedirs(OUTPUT_DIR, exist_ok=True)
print("Loading YOLOv8n ...")
model = YOLO("yolov8n.pt")
print("Model loaded.\n")

images = sorted(glob.glob(os.path.join(IMAGE_DIR, "*.jpg")))[:NUM_IMAGES]
if not images:
    print(f"ERROR: No images found in {IMAGE_DIR}")
    exit(1)
print(f"Testing on {len(images)} images\n")
print("=" * 60)

total_time = 0
for img_path in images:
    fname = os.path.basename(img_path)
    t0 = time.time()
    results = model(img_path, conf=CONFIDENCE, verbose=False)[0]
    elapsed_ms = (time.time() - t0) * 1000
    total_time += elapsed_ms
    boxes = results.boxes
    print(f"Image: {fname}  |  inference: {elapsed_ms:.1f}ms  |  detections: {len(boxes)}")
    for box in boxes:
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        x1,y1,x2,y2 = map(int, box.xyxy[0])
        orig_label = VISDRONE_CLASSES.get(cls_id, f"class_{cls_id}")
        remap_label = REMAP.get(cls_id, "other")
        print(f"  [{remap_label:7s}] {orig_label:16s} conf={conf:.2f}  box=({x1},{y1})-({x2},{y2})")
    annotated = results.plot()
    out_path = os.path.join(OUTPUT_DIR, fname.replace(".jpg", "_result.jpg"))
    cv2.imwrite(out_path, annotated)

print("=" * 60)
print(f"\nAverage inference time: {total_time/len(images):.1f}ms")
print(f"Estimated FPS: {1000/(total_time/len(images)):.1f}")
print(f"\nAnnotated images saved to: {OUTPUT_DIR}")
print("Done.")
