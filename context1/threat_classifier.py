class ThreatClassifier:
    """
    Classifies tracked objects as: static, pedestrian, vehicle, bird
    Based on bounding box size, altitude, and velocity.
    """
    def classify(self, bbox, velocity, altitude):
        """
        bbox: (x, y, w, h) in pixels
        velocity: (vx, vy) pixels/frame
        altitude: meters (negative = above ground in NED)
        returns: str classification
        """
        w, h = bbox[2], bbox[3]
        speed = (velocity[0]**2 + velocity[1]**2) ** 0.5
        area = w * h

        # bird: small bbox, high altitude, fast moving
        if area < 1000 and altitude > 20 and speed > 3:
            return "bird"

        # static: large bbox, low altitude, slow
        if area > 2000 and speed < 1.0:
            return "static"

        # vehicle: wide bbox
        if w > h * 1.5 and speed > 1.0:
            return "vehicle"

        # pedestrian: tall bbox
        if h > w and speed > 0.5:
            return "pedestrian"

        return "unknown"
