import sys
import skimage.io as skio
import skimage as sk
import skimage.transform as transform
import numpy as np
import settings
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import scipy.cluster.hierarchy as hcluster

def preprocess_grid(grid, flip=False):
    if flip:
        grid = grid[::-1]
    gsize = grid.shape
    grid = sk.measure.block_reduce(grid, (2,2), np.min)
    grid = transform.resize(grid, gsize, preserve_range=True)
    return grid

def obstacle_map(gridmap):
    copy = np.copy(gridmap)
    copy[copy == 127] = 255
    return copy < 105

def obstacle_indeices(obstacle_map):
    return np.nonzero(obstacle_map)

def show_obstacle(gridmap):
    black = np.zeros(gridmap.shape)
    black[obstacle_map(gridmap)] = 255
    return black

def grow_obstacle(gridmap):
    ob_map = obstacle_map(gridmap)
    copy = np.copy(gridmap)
    mask = np.copy(ob_map)
    step = 5
    for dx in range(-step, step+1):
        for dy in range(-step, step+1):
            roll = np.roll(np.roll(ob_map, dx, axis = 0), dy, axis = 1)
            mask = np.logical_or(roll, mask)
    copy[mask] = 0  
    return copy

def frontier(gridmap):
    growed = grow_obstacle(gridmap)
    gray = growed == 127
    black = growed ==0
    white = np.logical_not(np.logical_or(gray, black))
    level_map = white*255+gray*127
    canny = cv2.Canny(level_map.astype(np.uint8), 1, 1)
    
    xs, ys = np.nonzero(canny)
    trued = np.copy(canny)
    for i in range(len(xs)):
        x, y = xs[i], ys[i]
        if x > 0 and x < gridmap.shape[0]:
            if y > 0 and y < gridmap.shape[1]:
                blackarea = black[x-4:x+5,y-4:y+5]
                blackave = np.mean(blackarea)
                
                whitearea = white[x-2:x+3,y-2:y+3]
                whiteave = np.mean(whitearea)
                if not blackave < 0.1:
                    trued[x][y] = 0
                if not whiteave > 0.3:
                    trued[x][y] = 0
                
    return trued

def frontier_cluster(grid):
    grid = preprocess_grid(grid, True)
    # skio.imsave("frontier_finding.jpg", grid)
    ft = frontier(grid)
    coordinates = np.array(np.nonzero(ft))
    coordinates = np.transpose(coordinates)
    if not coordinates.size:
        return np.array([]), np.array([]), np.array([])
    clusters = hcluster.fclusterdata(coordinates, settings.cluster_threshold, criterion="distance")
    group_id, group_count = np.unique(clusters, return_counts=True)
    for i in group_id:
        if group_count[i - 1] < settings.cluster_filter:
            clusters[clusters == i] = -5

    filter_lst = []
    for i in range(len(clusters)):
        if clusters[i] == -5:
            filter_lst.append(i)

    clusters = np.delete(clusters, filter_lst)
    coordinates = np.delete(coordinates, filter_lst, axis=0)
    cx, cy = np.transpose(coordinates)
    return (cx, cy, clusters)


