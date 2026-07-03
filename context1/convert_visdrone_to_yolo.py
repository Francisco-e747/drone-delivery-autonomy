import os
import glob
from PIL import Image

# VisDrone category mapping (0-indexed, skip 0=ignored regions)
# VisDrone categories: 0=ignored, 1=pedestrian, 2=person, 3=bicycle,
# 4=car, 5=van, 6=truck, 7=tricycle, 8=awning-tricycle, 9=bus, 10=motorcycle
CATEGORY_MAP = {
    1: 0,   # pedestrian
    2: 1,   # person
    3: 2,   # bicycle
    4: 3,   # car
    5: 4,   # van
    6: 5,   # truck
    7: 6,   # tricycle
    8: 7,   # awning-tricycle
    9: 8,   # bus
    10: 9,  # motorcycle
}

def convert_split(img_dir, ann_dir, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    ann_files = sorted(glob.glob(os.path.join(ann_dir, "*.txt")))
    converted = 0
    skipped = 0

    for ann_path in ann_files:
        base = os.path.splitext(os.path.basename(ann_path))[0]
        img_path = os.path.join(img_dir, base + ".jpg")

        if not os.path.exists(img_path):
            skipped += 1
            continue

        img = Image.open(img_path)
        img_w, img_h = img.size

        yolo_lines = []
        with open(ann_path, "r") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split(",")
                if len(parts) < 6:
                    continue

                x, y, w, h = int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3])
                category = int(parts[5])

                # skip ignored regions (category 0) and empty (category 11)
                if category not in CATEGORY_MAP:
                    continue
                # skip invalid boxes
                if w <= 0 or h <= 0:
                    continue

                cls_id = CATEGORY_MAP[category]

                # convert to YOLO format (normalized center x,y,w,h)
                x_center = (x + w / 2) / img_w
                y_center = (y + h / 2) / img_h
                w_norm = w / img_w
                h_norm = h / img_h

                # clamp to [0,1]
                x_center = max(0, min(1, x_center))
                y_center = max(0, min(1, y_center))
                w_norm = max(0, min(1, w_norm))
                h_norm = max(0, min(1, h_norm))

                yolo_lines.append(f"{cls_id} {x_center:.6f} {y_center:.6f} {w_norm:.6f} {h_norm:.6f}")

        out_path = os.path.join(out_dir, base + ".txt")
        with open(out_path, "w") as f:
            f.write("\n".join(yolo_lines))
        converted += 1

    print(f"  Converted: {converted}, Skipped: {skipped}")

BASE = "../data/visdrone_samples"

print("Converting training set...")
convert_split(
    img_dir=f"{BASE}/VisDrone2019-DET-train/images",
    ann_dir=f"{BASE}/VisDrone2019-DET-train/annotations",
    out_dir=f"{BASE}/VisDrone2019-DET-train/labels"
)

print("Converting validation set...")
convert_split(
    img_dir=f"{BASE}/VisDrone2019-DET-val/images",
    ann_dir=f"{BASE}/VisDrone2019-DET-val/annotations",
    out_dir=f"{BASE}/VisDrone2019-DET-val/labels"
)

print("Done. Labels saved to labels/ folders.")
