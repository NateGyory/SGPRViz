from sklearn.metrics import precision_recall_curve, roc_curve, confusion_matrix
from sklearn.metrics import PrecisionRecallDisplay

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

def getTruthPred(path, truth_list, pred_list, method):
    file_list = os.listdir(path)
    for file_name in file_list:
        f = open(path + "/" + file_name)
        data = json.load(f)

        if file_name == "pr_no_loop.json" and method == 1:
            for i in range(50):
                truth_list.extend(data["truth"])
                pred_list.extend(data["pred"])

        truth_list.extend(data["truth"])
        pred_list.extend(data["pred"])


# Create the argument parser
parser = argparse.ArgumentParser(description='Example script for parsing a command line argument')

# Add the `seq` argument to the parser
parser.add_argument('--seq', help='The sequence to parse')

args = parser.parse_args()

seq = args.seq

path_100 = "/home/nate/Development/semnatickitteval/results/" + seq + "/100/0"
path_500 = "/home/nate/Development/semnatickitteval/results/" + seq + "/500/0"
path_1000 = "/home/nate/Development/semnatickitteval/results/" + seq + "/1000/1"

truth_list_100 = list()
pred_list_100 = list()
truth_list_500 = list()
pred_list_500 = list()
truth_list_1000 = list()
pred_list_1000 = list()

getTruthPred(path_100, truth_list_100, pred_list_100, 0)
getTruthPred(path_500, truth_list_500, pred_list_500, 0)
getTruthPred(path_1000, truth_list_1000, pred_list_1000, 1)

#confusion_pred = [ x >= .55 for x in pred_list]
#c = confusion_matrix(truth_list, confusion_pred)
#c = confusion_matrix(truth_list, confusion_pred)
#c = confusion_matrix(truth_list, confusion_pred)
#tn, fp, fn, tp = confusion_matrix(truth_list, confusion_pred).ravel()
#print(c)
#print("TP: ", tp)
#print("TN: ", tn)
#print("FP: ", fp)
#print("FN: ", fn)


# Plot PR and ROC curves
precision_100, recall_100, thresholds_100 = precision_recall_curve(truth_list_100, pred_list_100)
precision_500, recall_500, thresholds_500 = precision_recall_curve(truth_list_500, pred_list_500)
precision_1000, recall_1000, thresholds_1000 = precision_recall_curve(truth_list_1000, pred_list_1000)
#print(precision)
#print(recall)
#print(thresholds)


# TODO uncomment for AP
display1= PrecisionRecallDisplay.from_predictions(truth_list_100, pred_list_100, name="100", lw=3)
display2 = PrecisionRecallDisplay.from_predictions(truth_list_500, pred_list_500, name="500", lw=3)
display3= PrecisionRecallDisplay.from_predictions(truth_list_1000, pred_list_1000, name="1000", lw=3)

# Plot both PR curves on the same plot
ax = display1.ax_
#display1.plot(ax=ax)
display2.plot(ax=ax, lw=3)
display3.plot(ax=ax, lw=3)
ax.set_title('Precision-Recall Sample Size Comparison', fontsize = 20)
ax.set_xlabel('Recall', fontsize=20)
ax.xaxis.set_tick_params(labelsize=20)
ax.set_ylabel('Precision', fontsize=20)
ax.yaxis.set_tick_params(labelsize=20)
ax.set_ylim(0,1.1)
ax.legend()
ax.legend(prop=dict(size=15))

label_str1 = ax.get_lines()[0].get_label()
label_str2 = ax.get_lines()[1].get_label()
label_str3 = ax.get_lines()[2].get_label()
#label = seq + " (AP = {})".format(str(round(ap, 2)))
fig2 = plt.figure()
plt.plot(recall_100, precision_100, label=label_str1, lw=3)
plt.plot(recall_500, precision_500, label=label_str2, lw=3)
plt.plot(recall_1000, precision_1000, label=label_str3, lw=3)
ax2 = plt.gca()
ax2.set_xlabel('Recall', fontsize=20)
ax2.xaxis.set_tick_params(labelsize=20)
ax2.set_ylabel('Precision', fontsize=20)
ax2.yaxis.set_tick_params(labelsize=20)
ax2.set_title('Precision-Recall Sample Size Comparison', fontsize=20)
ax2.set_ylim(0,1.1)
ax2.legend()
ax2.legend(prop=dict(size=15))

# Add legend and show the plot
plt.show()

#display.ax_.plot(recall_500, precision_500, label='500')
#display.ax_.plot(recall_1000, precision_1000, label='1000')
#display = PrecisionRecallDisplay.from_predictions(truth_list_500, pred_list_500, name="500", lw=3)
#display = PrecisionRecallDisplay.from_predictions(truth_list_1000, pred_list_1000, name="1000", lw=3)
#display = PrecisionRecallDisplay.from_predictions(truth_list_100, pred_list_100, name="100", lw=3)
#_ = display.ax_.set_title("Precision-Recall curve")
#
#ax1 = plt.gca()
#ax1.plot(recall_100, precision_100, lw=3)
#ax1.plot(recall_500, precision_500, lw=3)
#ax1.plot(recall_1000, precision_1000, lw=3)
#ax1.set_xlabel('Recall', fontsize=20)
#ax1.xaxis.set_tick_params(labelsize=20)
#ax1.set_ylabel('Precision', fontsize=20)
#ax1.yaxis.set_tick_params(labelsize=20)
#ax1.set_title("Precision Recall Curve", fontsize=20)
#ax1.set_ylim(0,1.1)
#ax1.legend()
#ax1.legend(prop=dict(size=15))
#
#plt.show()
#f.close()
