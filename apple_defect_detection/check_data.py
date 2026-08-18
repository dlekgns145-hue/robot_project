import yaml, os
from collections import Counter

with open('Apple-Dataset-3/data.yaml') as f:
    data_yaml = yaml.safe_load(f)

print("클래스:", data_yaml['names'])

for split in ['train', 'valid', 'test']:
    label_dir = os.path.join('Apple-Dataset-3', split, 'labels')
    if not os.path.exists(label_dir):
        print(split, "폴더 없음")
        continue
    counter = Counter()
    for fname in os.listdir(label_dir):
        with open(os.path.join(label_dir, fname)) as f:
            for line in f:
                cls_id = int(line.split()[0])
                counter[cls_id] += 1
    print(split, {data_yaml['names'][k]: v for k, v in counter.items()})