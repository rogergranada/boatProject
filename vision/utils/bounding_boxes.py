#!/usr/bin/python
#-*- coding: utf-8 -*-

"""
Calculate Precision Recall and F-measure for a single image containing 
bounding boxes.
Ground truth and predicted files must have the format:

 
"""
import logging
logger = logging.getLogger(__name__)
logging.basicConfig(format='%(asctime)s : %(levelname)s : %(message)s', level=logging.INFO)
import argparse
import sys
from os import walk
from os.path import join, isdir, dirname, basename, splitext
from xml.dom import minidom
import json
import numpy as np

def ground_truth_xml_json(xml_folder, output=None):
    """
    Convert all xml files from a folder into a single json file.
    Output file has the format:
 
    {"image_1": {
        "classes": [class_bb1, class_bb2, ... class_bbN],
        "boxes": [[xmin, ymin, xmax, ymax],...,[xmin, ymin, xmax, ymax]]
     },
     "image_2": {
        "classes": [class_bb1, class_bb2, ... class_bbN],
        "boxes": [[xmin, ymin, xmax, ymax],...,[xmin, ymin, xmax, ymax]]
     },
     ...
    }    

    Parameters:
    -----------
    xml_folder: string
        path to the folder containing XML files
    output: string
        path to the folder to save the json file

    Return:
    -------
    dic : dictionary as a json file containing paths to images 
          and their respective bounding boxes. 
    """
    if not output or not isdir(output):
        output = dirname(xml_folder)
    fjson = join(output, 'ground_truth.json')

    dic = {}
    if not isdir(xml_folder): 
        logger.error("Input is not a folder")
        sys.exit(0)
    for root, dirs, files in walk(xml_folder):
        for name in sorted(files):
            xmlfile = minidom.parse(join(root, name))
            #xmlfile = minidom.parse('/home/roger/Downloads/resultados/GT/k_kitchen_100_1238.xml')
            if not xmlfile.getElementsByTagName('object'):
                continue
            itemlist = xmlfile.getElementsByTagName('filename')
            filename = itemlist[0].childNodes[0].data
            
            itemlist = xmlfile.getElementsByTagName('object')
            classes = []
            boxes = []
            for obj in itemlist:
                for elem in obj.childNodes:
                    if elem.tagName == 'name':
                        name = elem.childNodes[0].data
                        classes.append(name)
                    if elem.tagName == 'bndbox':
                        for item in elem.childNodes:
                            if item.tagName == 'xmin': xmin = int(item.childNodes[0].data)
                            if item.tagName == 'ymin': ymin = int(item.childNodes[0].data)
                            if item.tagName == 'xmax': xmax = int(item.childNodes[0].data)
                            if item.tagName == 'ymax': ymax = int(item.childNodes[0].data)
                        boxes.append([xmin, ymin, xmax, ymax])
            dic[filename] = {'classes': classes, 'boxes': boxes}

    logger.info('Saving file %s' % fjson)
    with open(fjson, 'w') as outfile:
        json.dump(dic, outfile)


def predictions_to_json(gt_file, classes_folder, output=None):
    """
    Convert a set of files containing prediction in the form:

    file: <name_of_the_class>
    <image> prediction xmin, ymin, xmax, ymax

    To a json file in the form:

    {"image_1": {
        "classes": [class_bb1, class_bb2, ... class_bbN],
        "boxes": [[xmin, ymin, xmax, ymax],...,[xmin, ymin, xmax, ymax]],
        "scores": [score_1,...,score_N]},
     "image_2": {
        "classes": [class_bb1, class_bb2, ... class_bbN],
        "boxes": [[xmin, ymin, xmax, ymax],...,[xmin, ymin, xmax, ymax]],
        "scores": [score_1,...,score_N]},
     ...
    }
    """
    if not output or not isdir(output):
        output = dirname(gt_file)
    fileout = basename(dirname(classes_folder))
    fjson = join(output, fileout+'.json')

    with open(gt_file) as infile:
        dgt = json.load(infile)

    dic = {}
    for root, dirs, files in walk(classes_folder):
        for name in sorted(files):
            label = name.replace('comp4_det_test_in_', '')
            label = label.replace('.txt', '')
            
            with open(join(root, name)) as fin:
                for line in fin:
                    arr = line.strip().split()
                    image = arr[0]+'.jpg'
                    score = float(arr[1])
                    xmin = int(float(arr[2]))
                    ymin = int(float(arr[3]))
                    xmax = int(float(arr[4]))
                    ymax = int(float(arr[5]))
                    if dgt.has_key(image):
                        if dic.has_key(image):
                            dic[image]['classes'].append(label)
                            dic[image]['boxes'].append([xmin, ymin, xmax, ymax])
                            dic[image]['scores'].append(score)
                        else:
                            dic[image] = {
                                'classes': [label],
                                'boxes': [[xmin, ymin, xmax, ymax]],
                                'scores': [score]
                            }
    logger.info('Saving file %s' % fjson)
    with open(fjson, 'w') as outfile:
        json.dump(dic, outfile)
                    

def accurary_scores(dresults):
    """
    Calculate the precision and recall of predicted bounding boxes

    Parameters:
    -----------
    dresults : dict
        Dictionary containing true positives, false positives and false negatives
        The dictionary has the form:
        {'true_pos': int, 'false_pos': int, 'false_neg': int}
    """
    tp = dresults['true_pos']
    fp = dresults['false_pos']
    fn = dresults['false_neg']
    try:
        precision = float(tp)/(tp + fp)
    except ZeroDivisionError:
        precision = 0.0
    try:
        recall = float(tp)/(tp + fn)
    except ZeroDivisionError:
        recall = 0.0
    try:
        f_score = 2*(precision * recall) / (precision + recall)
    except ZeroDivisionError:
        f_score = 0.0
    return (precision, recall, f_score)

####################
def calculate_iou(g_bbox, p_bbox):
    """
    Calculate Intersection over Union (IoU) of a pair of bounding boxes

    Parameters:
    -----------
        g_bbox list
            ground truth bounding box in the form [xmin, ymin, xmax, ymax]
        p_bbox: list
            predicted bounding box in the form [xmin, ymin, xmax, ymax]

    Returns:
    --------
        float: value of the IoU
    """
    g_xmin, g_ymin, g_xmax, g_ymax = g_bbox
    p_xmin, p_ymin, p_xmax, p_ymax = p_bbox

    if (g_xmin > g_xmax) or (g_ymin > g_bbox) or \
       (p_xmin > p_xmax) or (p_ymin > p_bbox):
        logger.error('Bounding box contain errors, e.g., xmin>max')
        sys.exit(0)

    if (g_xmax < p_xmin or p_xmax < g_xmin or \
        g_ymax < p_ymin or p_ymax < g_ymin):
        return 0.0

    far_x = np.min([g_xmax, p_xmax])
    near_x = np.max([g_xmin, p_xmin])
    far_y = np.min([g_ymax, p_ymax])
    near_y = np.max([g_ymin, p_ymin])

    inter_area = (far_x - near_x + 1) * (far_y - near_y + 1)
    true_box_area = (g_xmax - g_xmin + 1) * (g_ymax - g_ymin + 1)
    p_bbox_area = (p_xmax - p_xmin + 1) * (p_ymax - p_ymin + 1)
    iou = float(inter_area) / (true_box_area + p_bbox_area - inter_area)
    return iou


def get_single_image_results(gt_boxes, pred_boxes, iou_thr):
    """Calculates number of true_pos, false_pos, false_neg from single batch of boxes.

    Args:
        gt_boxes (list of list of floats): list of locations of ground truth
            objects as [xmin, ymin, xmax, ymax]
        pred_boxes (dict): dict of dicts of 'boxes' (formatted like `gt_boxes`)
            and 'scores'
        iou_thr (float): value of IoU to consider as threshold for a
            true prediction.

    Returns:
        dict: true positives (int), false positives (int), false negatives (int)
    """
    all_pred_indices = range(len(pred_boxes))
    all_gt_indices = range(len(gt_boxes))
    if len(all_pred_indices) == 0:
        tp = 0
        fp = 0
        fn = len(gt_boxes)
        return {'true_pos': tp, 'false_pos': fp, 'false_neg': fn}
    if len(all_gt_indices) == 0:
        tp = 0
        fp = len(pred_boxes)
        fn = 0
        return {'true_pos': tp, 'false_pos': fp, 'false_neg': fn}

    gt_idx_thr = []
    pred_idx_thr = []
    ious = []
    for ipb, pred_box in enumerate(pred_boxes):
        for igb, gt_box in enumerate(gt_boxes):
            iou = calculate_iou(pred_box, gt_box)
            if iou > iou_thr:
                gt_idx_thr.append(igb)
                pred_idx_thr.append(ipb)
                ious.append(iou)

    args_desc = np.argsort(ious)[::-1]
    if len(args_desc) == 0:
        # No matches
        tp = 0
        fp = len(pred_boxes)
        fn = len(gt_boxes)
    else:
        gt_match_idx = []
        pred_match_idx = []
        for idx in args_desc:
            gt_idx = gt_idx_thr[idx]
            pr_idx = pred_idx_thr[idx]
            # If the boxes are unmatched, add them to matches
            if (gt_idx not in gt_match_idx) and (pr_idx not in pred_match_idx):
                gt_match_idx.append(gt_idx)
                pred_match_idx.append(pr_idx)
        tp = len(gt_match_idx)
        fp = len(pred_boxes) - len(pred_match_idx)
        fn = len(gt_boxes) - len(gt_match_idx)

    return {'true_pos': tp, 'false_pos': fp, 'false_neg': fn}


######################
def select_by_class(dic):
    """ For each image, create a dictionary with {label: [[bbox1], [bbox2],...]}
    """
    dclass = {}
    for img in sorted(dic):
        dcontent = {}
        for label, bbox in zip(dic[img]['classes'], dic[img]['boxes']):
            if dcontent.has_key(label):
                dcontent[label].append(bbox)
            else:
                dcontent[label] = [bbox]
        dclass[img] = dcontent
    return dclass


def image_results(g_img, p_img):
    all_classes = set(g_img.keys()+p_img.keys())
    results = { 'false_pos': 0, 'true_pos': 0, 'false_neg': 0 }
    for label in all_classes:
        if not g_img.has_key(label): g_img[label] = []
        if not p_img.has_key(label): p_img[label] = []
        dres = get_single_image_results(g_img[label], p_img[label], 0.5)
        results['false_pos'] += dres['false_pos']
        results['true_pos'] += dres['true_pos']
        results['false_neg'] += dres['false_neg']
    return results

    
def generate_results(file_ground, file_pred, output=None):
    if not output:
        fname, _ = splitext(basename(file_pred))
        output = join(dirname(file_pred), 'scores_'+fname+'.txt')
    logger.info('Saving file %s' % output)
    fout = open(output, 'w')

    with open(file_ground) as infile:
        dgt = json.load(infile)
    dg = select_by_class(dgt)
    with open(file_pred) as infile:
        dpred = json.load(infile)
    dp = select_by_class(dpred)

    # g_: ground p_: predicted
    for id, img in enumerate(sorted(dp)):
        g_img = dg[img]
        p_img = dp[img]
        dresults = image_results(g_img, p_img)
        scores = accurary_scores(dresults)
        fout.write('%s %f %f %f\n' % (img, scores[0], scores[1], scores[2]))
    fout.close()
        

def select_above_threshold(file_pred, output=None, threshold=0.5):
    """ 
    Save a file containing only the predicted bounding boxes with
    probability above `threshold`.
    """
    if not output:
        dirin = dirname(file_pred)
        fname, _ = splitext(basename(file_pred))
        fjson = join(dirin, fname+'_'+str(threshold)+'.json')

    with open(file_pred) as infile:
        dpred = json.load(infile)

    dic = {}
    for img in sorted(dpred):
        dimg = dpred[img]
        for label, box, score in zip(dimg['classes'], dimg['boxes'], dimg['scores']):
            if score >= threshold:
                if dic.has_key(img):
                    dic[img]['classes'].append(label)
                    dic[img]['boxes'].append(box)
                    dic[img]['scores'].append(score)
                else:
                    dic[img] = {
                        'classes': [label],
                        'boxes': [box],
                        'scores': [score]
                    }
    logger.info('Saving file %s' % fjson)
    with open(fjson, 'w') as outfile:
        json.dump(dic, outfile)


def check_difference(f_faster, f_leanet, output=None):
    if not output:
        output = join(dirname(f_faster), 'difference.csv')
    fout = open(output, 'w')
    dic = {}
    with open(f_faster) as fin:
        for line in fin:
            img, p, r, f = line.strip().split()
            dic[img] = {'p': float(p), 'r': float(r), 'f': float(f)}
        
    with open(f_leanet) as fin:
        for line in fin:
            img, p, r, f = line.strip().split()
            if dic.has_key(img):
                dif_p = float(p) - dic[img]['p']
                dif_r = float(r) - dic[img]['r']
                dif_f = float(f) - dic[img]['f']
                fout.write('%s,%f,%f,%f\n' % (img, dif_p, dif_r, dif_f))
    fout.close()


def main(fground, folderpred):
    check_difference(fground, folderpred)
    #generate_results(fground, folderpred)
    #select_above_threshold(folderpred)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('groundtruth', metavar='file_ground', help='File containing ground truth for all images')
    parser.add_argument('predicted', metavar='file_pred', help='File containing predicted bounding boxes')
    args = parser.parse_args()

    main(args.groundtruth, args.predicted)
