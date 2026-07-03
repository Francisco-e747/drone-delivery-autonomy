import numpy as np
from collections import OrderedDict

class KalmanBoxTracker:
    count = 0
    def __init__(self, bbox):
        # state: [x, y, w, h, vx, vy, vw, vh]
        self.kf_mean = np.array([bbox[0], bbox[1], bbox[2], bbox[3], 0, 0, 0, 0], dtype=float)
        self.id = KalmanBoxTracker.count
        KalmanBoxTracker.count += 1
        self.hits = 1
        self.no_hit_streak = 0
        self.age = 0

    def predict(self):
        self.kf_mean[0] += self.kf_mean[4]
        self.kf_mean[1] += self.kf_mean[5]
        self.age += 1
        self.no_hit_streak += 1
        return self.kf_mean[:4].copy()

    def update(self, bbox):
        alpha = 0.5
        self.kf_mean[4] = alpha * (bbox[0] - self.kf_mean[0]) + (1 - alpha) * self.kf_mean[4]
        self.kf_mean[5] = alpha * (bbox[1] - self.kf_mean[1]) + (1 - alpha) * self.kf_mean[5]
        self.kf_mean[:4] = bbox
        self.hits += 1
        self.no_hit_streak = 0

    def get_state(self):
        return self.kf_mean[:4].copy()


def iou(bb1, bb2):
    x1 = max(bb1[0], bb2[0])
    y1 = max(bb1[1], bb2[1])
    x2 = min(bb1[0]+bb1[2], bb2[0]+bb2[2])
    y2 = min(bb1[1]+bb1[3], bb2[1]+bb2[3])
    inter = max(0, x2-x1) * max(0, y2-y1)
    union = bb1[2]*bb1[3] + bb2[2]*bb2[3] - inter
    return inter / union if union > 0 else 0


class SORTTracker:
    def __init__(self, max_age=5, min_hits=2, iou_threshold=0.3):
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_threshold = iou_threshold
        self.trackers = []
        self.frame_count = 0

    def update(self, detections):
        self.frame_count += 1
        predicted = [t.predict() for t in self.trackers]

        # greedy IoU matching
        matched = set()
        det_matched = set()
        for ti, pred in enumerate(predicted):
            best_iou = self.iou_threshold
            best_di = -1
            for di, det in enumerate(detections):
                if di in det_matched:
                    continue
                if iou(pred, det) > best_iou:
                    best_iou = iou(pred, det)
                    best_di = di
            if best_di >= 0:
                self.trackers[ti].update(detections[best_di])
                matched.add(ti)
                det_matched.add(best_di)

        # new trackers for unmatched detections
        for di, det in enumerate(detections):
            if di not in det_matched:
                self.trackers.append(KalmanBoxTracker(det))

        # remove dead trackers
        self.trackers = [t for t in self.trackers if t.no_hit_streak <= self.max_age]

        # return active tracks
        return [
            (*t.get_state(), t.id)
            for t in self.trackers
            if t.hits >= self.min_hits or self.frame_count <= self.min_hits
        ]
