from sklearn.metrics import precision_recall_curve, roc_curve, confusion_matrix
from sklearn.metrics import PrecisionRecallDisplay
from sklearn.metrics import average_precision_score

import matplotlib.pyplot as plt
import json
import numpy as np
import argparse
import os

# JSON format for PR results
#{
#  results : [
#    ref_scan_id: scan_id
#    query_scan_truth: [... 0 or 1 for correct pred]
#    query_scan_pred: [... the pred percent]
#  ]
#}


# Create the argument parser
parser = argparse.ArgumentParser(description='Example script for parsing a command line argument')

# Add the `seq` argument to the parser
parser.add_argument('--seq', help='The sequence to parse')
parser.add_argument('--samp', help='The sample size')
parser.add_argument('--m', help='Method 0: all 1: max prob')

# Parse the command line arguments
args = parser.parse_args()

# Access the `seq` argument value
seq = args.seq
samp = args.samp
method = args.m

path = "/home/nate/Development/semnatickitteval/results/" + seq + "/" + samp + "/" + method

file_list = os.listdir(path)

truth_list = list()
pred_list = list()

for file_name in file_list:

    f = open(path + "/" + file_name)
    data = json.load(f)

    if file_name == "pr_no_loop.json" and method == '1':
        for i in range(50):
            truth_list.extend(data["truth"])
            pred_list.extend(data["pred"])

    truth_list.extend(data["truth"])
    pred_list.extend(data["pred"])


confusion_pred = [ x >= .55 for x in pred_list]
c = confusion_matrix(truth_list, confusion_pred)
c = confusion_matrix(truth_list, confusion_pred)
c = confusion_matrix(truth_list, confusion_pred)
tn, fp, fn, tp = confusion_matrix(truth_list, confusion_pred).ravel()
print(c)
print("TP: ", tp)
print("TN: ", tn)
print("FP: ", fp)
print("FN: ", fn)


# Plot PR and ROC curves
precision, recall, thresholds = precision_recall_curve(truth_list, pred_list)
#print(precision)
#print(recall)
#print(thresholds)

title = "Precision-Recall Seq: {} Samp: {}".format(seq, samp)

# TODO uncomment for AP
display = PrecisionRecallDisplay.from_predictions(truth_list, pred_list, name=seq, lw=3)
_ = display.ax_.set_title(title)

ax1 = plt.gca()
#ax1.plot(recall, precision, lw=3)
ax1.set_xlabel('Recall', fontsize=20)
ax1.xaxis.set_tick_params(labelsize=20)
ax1.set_ylabel('Precision', fontsize=20)
ax1.yaxis.set_tick_params(labelsize=20)
ax1.set_title(title, fontsize=20)
ax1.set_ylim(0,1.1)
ax1.legend()
ax1.legend(prop=dict(size=15))

label_str = ax1.get_lines()[0].get_label()
print("Label string: {}".format(label_str))
#label = seq + " (AP = {})".format(str(round(ap, 2)))
fig2 = plt.figure()
plt.plot(recall, precision, label=label_str, lw=3)
ax2 = plt.gca()
ax2.set_xlabel('Recall', fontsize=20)
ax2.xaxis.set_tick_params(labelsize=20)
ax2.set_ylabel('Precision', fontsize=20)
ax2.yaxis.set_tick_params(labelsize=20)
ax2.set_title(title, fontsize=20)
ax2.set_ylim(0,1.1)
ax2.legend()
ax2.legend(prop=dict(size=15))

plt.show()
f.close()
