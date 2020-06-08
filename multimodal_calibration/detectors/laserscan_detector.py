import numpy as np

def project_laserscan(ranges, angle_min, angle_max):
    N = len(ranges)
    t = np.linspace(angle_min, angle_max, N)
    
    return np.stack([
        ranges * np.cos(t),
        ranges * np.sin(t),
    ], 1)

def segment_ranges(ranges, delta, threshold):
    def distance(r1, r2):
        return np.sqrt(r1 * r1 + r2 * r2 - 2 * r1 * r2 * np.cos(delta))

    clusters = []
    
    N = len(ranges)
    left = 0
    prev = 0
    for i in range(N):
        if np.isnan(ranges[i]): continue
        
        r1 = ranges[prev]
        r2 = ranges[i]
        
        if distance(r1, r2) > threshold:
            clusters.append((left, prev + 1))
            left = i
        
        prev = i
    
    clusters.append((left, N))
    
    return clusters

def cluster_size(cluster):
    left, right = cluster
    return right - left

def iav_features(points):
    N = points.shape[0]
    if N < 3:
        raise ValueError('iav_features requires at least 3 points')
    
    p1 = points[0]
    pn = points[-1]
    ps = points[1:-1]
    
    v = ps - p1
    vn = v.T / np.sqrt(np.sum(v * v, 1))
    u = ps - pn
    un = u.T / np.sqrt(np.sum(u * u, 1))
    
    angles = np.arccos(np.sum(vn * un, 0))
    
    return (np.mean(angles), np.std(angles))

def is_circle(points):
    mean, stddev = iav_features(points)
    
    return (mean > (90 * np.pi / 180) and
            mean < (130 * np.pi / 180) and
            stddev < 0.2)

def fit_circle(points):
    center = np.mean(points, 0)
    uv = points - center
    u, v = uv.T
    
    Suu = np.sum(u * u)
    Svv = np.sum(v * v)
    Suv = np.sum(u * v)
    Suuu = np.sum(u * u * u)
    Suvv = np.sum(u * v * v)
    Svvv = np.sum(v * v * v)
    Svuu = np.sum(v * u * u)
    
    A = (np
         .array([Suu, Suv, Suv, Svv])
         .reshape(2,2))
    b = np.array([
            0.5 * (Suuu + Suvv),
            0.5 * (Svvv + Svuu)])

    uv_ = np.linalg.solve(A, b)
    
    center = center + uv_
    
    return center

class LaserscanDetector:

    def __init__(self, threshold, min_points):
        self.threshold = threshold
        self.min_points = min_points
    
    def detect(self, ranges, angles):
        angle_min, angle_max = angles
        N = len(ranges)
        delta = (angle_max - angle_min) / (N - 1)

        points = project_laserscan(ranges, angle_min, angle_max)

        clusters = segment_ranges(ranges, delta, self.threshold)
        clusters = sorted(clusters, key=cluster_size, reverse=True)

        cluster_pts = [
            points[left:right, :]
            for left, right in clusters
            if (right - left) > self.min_points]
        
        cluster_pts = filter(is_circle, cluster_pts)
        cluster_pts = list(cluster_pts)

        if len(cluster_pts) == 0:
            return
        
        points = cluster_pts[0]
        center = fit_circle(points)

        vecs = points - center
        radius = np.mean(np.sqrt(np.sum(vecs * vecs, 1)))

        return center, radius


