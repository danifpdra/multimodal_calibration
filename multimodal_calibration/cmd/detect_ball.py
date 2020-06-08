import cv2
import numpy as np

from multimodal_calibration.detectors import (CameraDetector, CameraMapper)

DEFAULT_PARAMS = dict(
    hue_max=20,
    hue_min=170,
    saturation=150,
    value=30,
    kernel_size=5,
    min_area=8000,
    flimits=(0.10, 0.15))

def read_all_images(path):
    from pathlib import Path

    path = Path(path)
    images = path.glob('*.png')
    images = sorted(images)

    return images

def detect_ball(detector, img):
    timestamp = img.name[:-4]

    im = cv2.imread(str(img))
    rgb = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)

    _, detection = detector.detect(rgb)

    if detection is not None:
        return timestamp, detection
    else:
        return None

def process_images_sequencial(detector, images):
    detections = []
    timestamps = []
    for img in tqdm(images):
        d = detect_ball(detector, img)
        if d is not None:
            timestamp, detection = d
            timestamps.append(timestamp)
            detections.append(detection)
    
    return timestamps, detections

def process_images_parallel(detector, images):
    import dask.bag as db
    from dask.diagnostics import ProgressBar
    ProgressBar().register()

    d = (db
        .from_sequence(images)
        .map(lambda img: detect_ball(detector, img))
        .filter(lambda r: r is not None))

    timestamps, detections = zip(*d)
    timestamps = list(timestamps)
    detections = list(detections)

    return timestamps, detections

def create_annotated_video(detector, images, out, fps=30, size=(964, 724)):
    video_writer = cv2.VideoWriter('./video.mp4', cv2.VideoWriter_fourcc(*'mp4v'), float(fps), size)

    for img in tqdm(images):
        im = cv2.imread(str(img))
        rgb = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
        
        _, detection = bd.detect(rgb)

        if detection is not None:
            (x, y), r = detection
            
            im = cv2.circle(im, (int(x), int(y)), int(r), (0, 255, 0), 4)
        
        video_writer.write(im)
    
    video_writer.release()

def transform_2d_into_3d(mapper, detections):
    pass

if __name__ == "__main__":
    print("detect_ball")
