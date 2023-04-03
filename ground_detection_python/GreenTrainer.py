import cv2
import numpy as np
import glob

from random import randrange

images = glob.glob( './curtain_open_precise/*c.jpg', recursive=True)
labels = glob.glob( './curtain_open_precise/*m.jpg', recursive=True)

print(images)

X_vec = []
y_vec = []

maxfiles = 500
samples_per_image = 120000

for f in images:
    lf = f.replace('c.jpg', 'm.jpg')
    if lf in labels:
        maxfiles = maxfiles - 1
        if maxfiles <= 0:
            break

        img = cv2.imread(f)
        msk = cv2.imread(lf)
        h, w, d = img.shape

        print('img=', f, 'lbl=', lf, w, 'x', h)

        # Color space of RAW Bebop Images
        yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
        img[:, :, 1] = msk[:, :, 0]

        for i in range(0, samples_per_image):

            x = randrange(2, w - 3)
            y = randrange(4, h - 2)

            # Pixels
            p = yuv[y, x]
            # Ground thruth
            m = int(msk[y, x, 0])
            if m < 127:
                m = 0
            else:
                m = 255

            # Y, U, V
            X_vec.append([int(p[0]), int(p[1]), int(p[2])])
            y_vec.append([m])

print('Dataset', len(X_vec), len(y_vec))

from sklearn.model_selection import train_test_split


X_train, X_test, y_train, y_test = train_test_split(X_vec, y_vec, test_size=0.2, stratify=y_vec, random_state=1)

print('Train',len(X_train), len(y_train))
print('Test',len(X_test), len(y_test))

from sklearn.tree import DecisionTreeClassifier
from sklearn.metrics import accuracy_score

dt = DecisionTreeClassifier(max_depth=6, random_state=0)
dt.fit(X_train, y_train)

y_pred = dt.predict(X_test)

score = accuracy_score(y_test, y_pred)
print('Sensitivity:',round(score,3))

from sklearn.tree import export_text

text_representation = export_text(dt, feature_names=['Y','U','V'])

print(text_representation)

from matplotlib import pyplot as plt


test_images = glob.glob( './test_open/*.jpg', recursive=True)
count = 1

#show training images
for f in images:
    img = cv2.imread(f)
    h, w, d = img.shape

    # Load an image
    yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)

    X_run = yuv.reshape(int(h * w), int(d))
    y_pred = dt.predict(X_run)
    msk = y_pred.reshape(h, w)

    img[:, :, 1] = msk[:, :]
    #cv2.imwrite('./test_open/original_'+str(count)+'.jpg', img)
    count += 1
    plt.imshow(img)
    plt.title('Training figure ' + str(f))
    plt.show()

#show test images to see if classifier is acurate
for f in test_images:
    img = cv2.imread(f)
    h, w, d = img.shape

    # Load an image
    yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)

    X_run = yuv.reshape(int(h * w), int(d))
    y_pred = dt.predict(X_run)
    msk = y_pred.reshape(h, w)

    img[:, :, 1] = msk[:, :]

    plt.imshow(img)
    #cv2.imwrite('./new_roc/original_' + str(count) + '.jpg', img)
    count += 1
    plt.title('Test figure ' + str(f))
    plt.show()

from sklearn.tree import _tree



#This part of the code writes the C code that uses the correct classifier values
code_snippet = '''{
\t\t\t\t\t\t\t\tif (draw){
\t\t\t\t\t\t\t\t\t*yp = 255;  // make pixel brighter in image
\t\t\t\t\t\t\t\t}
\t\t\t\t\t\t\t\tkernel_cnt++;
\t\t\t\t\t\t\t}'''
tabs = '\t\t\t\t\t\t\t'

def get_rules(tree, feature_names, class_names):
    tree_ = tree.tree_
    feature_name = [
        feature_names[i] if i != _tree.TREE_UNDEFINED else "undefined!"
        for i in tree_.feature
    ]

    paths = []
    path = []

    def recurse(node, path, paths):

        if tree_.feature[node] != _tree.TREE_UNDEFINED:
            name = feature_name[node]
            threshold = tree_.threshold[node]
            p1, p2 = list(path), list(path)
            p1 += [f"({name} <= {np.round(threshold, 3)})"]
            recurse(tree_.children_left[node], p1, paths)
            p2 += [f"({name} > {np.round(threshold, 3)})"]
            recurse(tree_.children_right[node], p2, paths)
        else:
            path += [(tree_.value[node], tree_.n_node_samples[node])]
            paths += [path]

    recurse(0, path, paths)

    # sort by samples count
    samples_count = [p[-1][1] for p in paths]
    ii = list(np.argsort(samples_count))
    paths = [paths[i] for i in reversed(ii)]

    rules = []
    count = 0
    for path in paths:
        classes = path[-1][0][0]
        l = np.argmax(classes)
        if l == 1:
            rule = tabs
            rule += "if( "

            for p in path[:-1]:
                if rule != tabs+"if( ":
                    rule += " && "
                rule += str(p)
            rule += " )"
            rule += code_snippet
            rules += [rule]

    return rules

rules = get_rules(dt, feature_names=['*yp','*up','*vp'], class_names=['Not green', 'Green'])
for r in rules:
    print(r)

