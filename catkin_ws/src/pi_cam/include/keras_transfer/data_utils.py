#!coding:utf-8
import numpy as np
import os
# import cv2
import pickle
# from tensorflow.keras.applications.vgg16 import preprocess_input
from tensorflow.keras.preprocessing.image import load_img, img_to_array
import tensorflow.keras.backend as K
import cv2

import sys  # 引用sys模块进来，并不是进行sys的第一次加载
reload(sys)  # 重新加载sys
sys.setdefaultencoding('utf8')  # 调用setdefaultencoding函数


def get_labels(data_dir):
    """
    加载标签
    """
    labels = []
    files = os.listdir(data_dir)
    files.sort()
    for file in files:
        if not os.path.isdir(os.path.join(data_dir, file)):
            continue
        catname_full, _ = os.path.splitext(file)
        # catname = catname_full.split('_')[-1]
        # if len(os.listdir(os.path.join(data_dir,file)))>=1:
        labels.append(catname_full)
    print(labels)
    labels.sort()
    return labels


def load_img_from_dir(data_dir, target_size=(112, 112), max_num=100):  # From Directory
    """
    从目录加载图片并缩放至指定尺寸
    """
    x_load = []
    y_load = []
    labels = get_labels(data_dir)
    dirs = labels
    for cat in dirs:  # load directory
        files_dir = os.path.join(data_dir, cat)
        files = os.listdir(files_dir)
        for file in files[:max_num]:
            file_path = os.path.join(files_dir, file)
            try:
                cv_img = cv2.imread(file_path)
                # cv_img=cv2.imdecode(np.fromfile(file_path,dtype=np.uint8),-1)
                x = img_to_array(cv2.resize(cv_img, target_size))
                # x = K.expand_dims(x, axis=0)
                # x = preprocess_input(x)
                x_load.append(x)
                y_load.append(labels.index(cat))  # directory name as label
            except Exception as e:
                print(e)
                continue
    return np.array(x_load), np.array(y_load)


def dump_picle(features, labels):
    """
    保存为picle文件
    """
    features = np.array(features).astype('float32')
    labels = np.array(labels).astype('float32')
    print(features.shape)
    print(labels.shape)
    features = features.reshape(
        features.shape[0]*features.shape[1], features.shape[2])
    labels = labels.reshape(labels.shape[0]*labels.shape[1], labels.shape[2])
    print(features.shape)
    print(labels.shape)
    with open("features", "wb") as f:
        pickle.dump(features, f, protocol=4)
    with open("labels", "wb") as f:
        pickle.dump(labels, f, protocol=4)


def loadFromPickle():
    """
    从picle文件加载
    """
    with open("features", "rb") as f:
        features = np.array(pickle.load(f))
    with open("labels", "rb") as f:
        labels = np.array(pickle.load(f))
    return features, labels

def dump_label_name(dirs,name="label_names"):
    """
    保存标签名，因为模型不会记录标签
    """
    with open(name, "wb") as f:
        pickle.dump(dirs, f, protocol=4)


def load_label_name(name="label_names"):
    """
    加载标签名
    """
    with open(name, "rb") as f:
        dirs = np.array(pickle.load(f))
    return dirs


def prepress_labels(labels):
    """
    # one-hot编码 把类别id转换为表示当前类别的向量，比如0 1 2 =》 [[1 0 0] [0 1 0] [0 0 1]]
    """
    from tensorflow.keras import utils
    labels = utils.to_categorical(labels)
    return labels


if __name__ == '__main__':
    get_labels(os.path.join("re", "train"))
