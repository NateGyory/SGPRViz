import json
import argparse

parser = argparse.ArgumentParser()

parser.add_argument('--seq', type=str, required=True, help='Sequence string')
parser.add_argument('--samp', type=str, required=True, help='Sample string')

args = parser.parse_args()

seq = args.seq
samp = args.samp

with open('/home/nate/Development/semnatickitteval/results/' + seq + "/" + samp + "/1/pr_no_loop.json") as f:
    data = json.load(f)

# Extract the list of floats from the "pred" tag
pred_list = data['pred']

# Find the max value in the list
max_value = max(pred_list)

# Print out the max value
print("The maximum value in the 'pred' list is:", max_value)
