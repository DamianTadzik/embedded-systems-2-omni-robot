import numpy as np

# Vibe coded section begins

def get_bbox_distance_percentile(depth_frame, x1, y1, x2, y2, percentile=35):
    depth_image = np.asanyarray(depth_frame.get_data())

    h, w = depth_image.shape
    x1 = max(0, min(x1, w-1))
    x2 = max(0, min(x2, w-1))
    y1 = max(0, min(y1, h-1))
    y2 = max(0, min(y2, h-1))

    if x1 >= x2:
        x1, x2 = x2, x1
    if y1 >= y2:
        y1, y2 = y2, y1

    roi = depth_image[y1:y2, x1:x2].astype(float)
    roi = roi[roi > 0]

    if len(roi) == 0:
        return  0.0

    # for i in range(5, 70, 5):
    #     print(f"distance {i}% is  {np.percentile(roi, i) / 1000}")
    distance_val = np.percentile(roi, percentile)

    return distance_val / 1000.0  # Convert to meters

# Vibe coded section ends