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
                    

def prec_rec(gt_file, pred_file):
    """
    Calculate the precision and recall of predicted bounding boxes

    Parameters:
    -----------
    gt_file : string
        json file containing ground truth
    pred_file : string 
        json file containing predicted bounding boxes
    """
    pass

####################3
def iou(pred_box, gt_box):
    """Calculate IoU of single predicted and ground truth box

    Args:
        pred_box (list of floats): location of predicted object as
            [xmin, ymin, xmax, ymax]
        gt_box (list of floats): location of ground truth object as
            [xmin, ymin, xmax, ymax]

    Returns:
        float: value of the IoU for the two boxes.

    Raises:
        AssertionError: if the box is obviously malformed
    """
    print '>>', pred_box, gt_box
    x1_t, y1_t, x2_t, y2_t = gt_box
    x1_p, y1_p, x2_p, y2_p = pred_box

    if (x1_p > x2_p) or (y1_p > y2_p):
        raise AssertionError(
            "Prediction box is malformed? pred box: {}".format(pred_box))
    if (x1_t > x2_t) or (y1_t > y2_t):
        raise AssertionError(
            "Ground Truth box is malformed? true box: {}".format(gt_box))

    if (x2_t < x1_p or x2_p < x1_t or y2_t < y1_p or y2_p < y1_t):
        return 0.0

    far_x = np.min([x2_t, x2_p])
    near_x = np.max([x1_t, x1_p])
    far_y = np.min([y2_t, y2_p])
    near_y = np.max([y1_t, y1_p])

    inter_area = (far_x - near_x + 1) * (far_y - near_y + 1)
    print '#',inter_area
    true_box_area = (x2_t - x1_t + 1) * (y2_t - y1_t + 1)
    pred_box_area = (x2_p - x1_p + 1) * (y2_p - y1_p + 1)
    iou = float(inter_area) / (true_box_area + pred_box_area - inter_area)
    print 'iou', iou
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
            iou = calc_iou_individual(pred_box, gt_box)
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
    for label in all_classes:
        v = get_single_image_results(g_img[label], p_img[label], 0.2)
        print g_img[label]
        print p_img[label]
        print label, v

    
def generate_results(file_ground, file_pred):
    with open(file_ground) as infile:
        dgt = json.load(infile)
    dg = select_by_class(dgt)
    with open(file_pred) as infile:
        dpred = json.load(infile)
    dp = select_by_class(dpred)

    # g_: ground p_: predicted
    for img in sorted(dp):
        g_img = dg[img]
        p_img = dp[img]
        image_results(g_img, p_img)
        print img
        break
        
    """
    
        {u'boxes': [[32, 213, 49, 232], [221, 41, 238, 64], [234, 249, 250, 306], [189, 280, 206, 318], [116, 74, 196, 221]], u'classes': [u'clock', u'person', u'person', u'person', u'stop sign'], u'scores': [0.625, 0.113, 0.11, 0.059, 0.057]}
        {u'boxes': [[57, 215, 66, 226], [35, 214, 44, 224]], u'classes': [u'clock', u'clock']}

        
        break
    print p_img
    print g_img
    """


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
            

def main(fground, folderpred):
    #generate_results(fground, folderpred)
    generate_results(fground, folderpred)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('groundtruth', metavar='file_ground', help='File containing ground truth for all images')
    parser.add_argument('predicted', metavar='file_pred', help='File containing predicted bounding boxes')
    args = parser.parse_args()

    main(args.groundtruth, args.predicted)
