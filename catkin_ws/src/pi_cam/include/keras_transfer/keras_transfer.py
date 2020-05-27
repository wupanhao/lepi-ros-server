#!coding:utf-8
import tensorflow as tf
from tensorflow import keras
import os
import h5py
from data_utils import get_labels, prepress_labels, load_img_from_dir
from tensorflow.keras.models import load_model
from tensorflow.keras.preprocessing.image import load_img, img_to_array
import tensorflow.keras.backend as K
from sklearn.model_selection import train_test_split
from tensorflow.keras.models import Model
from tensorflow.keras.layers import GlobalAveragePooling2D, Dense
from tensorflow.keras.optimizers import SGD
import cv2
import pickle
import sys  # 引用sys模块进来，并不是进行sys的第一次加载
reload(sys)  # 重新加载sys
sys.setdefaultencoding('utf8')  # 调用setdefaultencoding函数


class AccuracyLogger(keras.callbacks.Callback):
    """
    AccuracyLogger 类, 用来记录训练进度
    Attributes:
    callback: function 回调函数
    epoch: int 回合
    batch: int 批次
    """

    def __init__(self, callback=None):
        self.callback = callback
        self.epoch = 0
        self.batch = 0

    def set_model(self, model):
        """
        set_model 函数, 在训练之前由父模型调用，告诉回调函数是哪个模型在调用它
        Keyword arguments::
        model: model 训练模型
        """
        self.model = model  #

    def on_epoch_begin(self, epoch, logs={}):
        """
        on_epoch_begin 函数, epoch开始的时候自动调用
        Keyword arguments::
        epoch: int 回合数
        """
        self.epoch = epoch
        # print('-----------------')
        # print(logs,epoch)
        # print('-----------------')

    def on_batch_end(self, batch, logs={}):
        """
        on_batch_end 函数, 在batch训练结束后自动调用
        Keyword arguments::
        batch: int 批次数
        """
        self.batch = batch
        print('epoch:%d batch:%d' % (self.epoch, self.batch))
        # print(logs)
        if self.callback is not None:
            self.callback(self.epoch, self.batch, logs)


class ImageClassifier:
    """
    ImageClassifier 类, 用来训练分类器
    Attributes:
    data_root: str 数据根目录
    FC_NUMS: int 回合
    FREEZE_LAYERS: int 批次
    model: model 模型
    busy: bool 是否正在执行操作
    ns: str 单次训练命名空间
    """

    def __init__(self, model_path=None, data_root=os.path.expanduser('~')+'/Lepi_Data/ros/transfer_learning'):
        self.data_root = data_root
        self.FC_NUMS = 64
        self.FREEZE_LAYERS = 17
        self.IMAGE_SIZE = 224
        self.model = None
        self.busy = False
        self.ns = None

    def load_model(self, path):
        """
        on_batch_end 函数, 在batch训练结束后自动调用
        Keyword arguments::
        batch: int 批次数
        """
        self.busy = True
        try:
            if self.model is None:
                self.model = load_model(path)  # 加载训练模型
            else:
                K.clear_session()
                self.model = load_model(path)
            self.model._make_predict_function()
            self.session = K.get_session()
            self.graph = tf.get_default_graph()
            self.graph.finalize()
        except Exception as e:
            print(e)
        finally:
            self.busy = False

    def download_model(self):
        """
        下载模型
        """
        from tensorflow.keras.applications.vgg16 import VGG16, preprocess_input
        base_model = VGG16(input_shape=(
            self.IMAGE_SIZE, self.IMAGE_SIZE, 3), include_top=False, weights='imagenet')

    def train(self, data_dir, epochs=3, callback=None):
        """
        train 函数, 训练模型
        Keyword arguments::
        epochs: int 训练次数
        """
        if self.busy:
            return 1
        self.busy = True
        label_names = get_labels(data_dir)
        self.NUM_CLASSES = len(label_names)
        from tensorflow.keras.applications.vgg16 import VGG16, preprocess_input
        # 采用VGG16为基本模型，include_top为False，表示FC层是可自定义的，抛弃模型中的FC层；该模型会在~/.keras/models下载基本模型
        base_model = VGG16(input_shape=(
            self.IMAGE_SIZE, self.IMAGE_SIZE, 3), include_top=False, weights='imagenet')
        x_data, y_label = load_img_from_dir(data_dir, target_size=(
            self.IMAGE_SIZE, self.IMAGE_SIZE), max_num=30)
        for i in range(x_data.shape[0]):
            x_data[i] = preprocess_input(x_data[i])
        print(x_data.shape)
        print(x_data[0].shape)
        x_data = x_data.reshape(
            x_data.shape[0], self.IMAGE_SIZE, self.IMAGE_SIZE, 3)
        y_label_one_hot = prepress_labels(y_label)
        # 验证应该使用从未见过的图片
        train_x, test_x, train_y, test_y = train_test_split(x_data, y_label_one_hot, random_state=0,
                                                            test_size=0.3)
        # 自定义FC层以基本模型的输入为卷积层的最后一层
        x = base_model.output
        x = GlobalAveragePooling2D()(x)
        x = Dense(self.FC_NUMS, activation='relu')(x)
        prediction = Dense(self.NUM_CLASSES, activation='softmax')(x)

        # 构造完新的FC层，加入custom层
        model = Model(inputs=base_model.input, outputs=prediction)

        # 获取模型的层数
        print("layer nums:", len(model.layers))

        # 除了FC层，靠近FC层的一部分卷积层可参与参数训练，
        # 一般来说，模型结构已经标明一个卷积块包含的层数，
        # 在这里我们选择FREEZE_LAYERS为17，表示最后一个卷积块和FC层要参与参数训练
        for layer in model.layers[:self.FREEZE_LAYERS]:
            layer.trainable = False
        for layer in model.layers[self.FREEZE_LAYERS:]:
            layer.trainable = True
        for layer in model.layers:
            print("layer.trainable:", layer.trainable)

        # 预编译模型
        model.compile(optimizer=SGD(lr=0.0001, momentum=0.9),
                      loss='categorical_crossentropy', metrics=['accuracy'])
        model.summary()
        model.fit(train_x, train_y,
                  validation_data=(test_x, test_y),
                  # model.fit(x_data, y_label_one_hot,
                  #         validation_split=0.4,
                  callbacks=[AccuracyLogger(callback)],
                  epochs=epochs, batch_size=4,
                  # steps_per_epoch=1,validation_steps =1 ,
                  verbose=1, shuffle=True)
        self.model = model
        model.save(os.path.join(data_dir, 'model.h5'))
        self.dump_label_name(label_names)
        self.session = K.get_session()
        self.graph = tf.get_default_graph()
        self.busy = False

    def dump_label_name(self, dirs):
        """
        保存训练标签
        """
        with open(os.path.join(self.data_root, self.ns, "label_names.txt"), "wb") as f:
            pickle.dump(dirs, f, protocol=2)

    def load_label_name(self):
        """
        加载训练标签
        """
        with open(os.path.join(self.data_root, self.ns, "label_names.txt"), "rb") as f:
            dirs = np.array(pickle.load(f))
        return dirs

    def evaluate(self, data_dir):
        """
        模型评估函数
        """
        x_data, y_label = load_img_from_dir(data_dir, target_size=(
            self.IMAGE_SIZE, self.IMAGE_SIZE), max_num=60)
        for i in range(x_data.shape[0]):
            x_data[i] = preprocess_input(x_data[i])
        x_data = x_data.reshape(
            x_data.shape[0], self.IMAGE_SIZE, self.IMAGE_SIZE, 3)
        y_label_one_hot = prepress_labels(y_label)
        # 验证应该使用从未见过的图片
        train_x, test_x, train_y, test_y = train_test_split(x_data, y_label_one_hot, random_state=0,
                                                            test_size=0.5)
        # 开始评估模型效果 # verbose=0为不输出日志信息
        score = model.evaluate(test_x, test_y, verbose=1, steps=1)
        print('Test loss:', score[0])
        print('Test accuracy:', score[1])  # 准确度

    def predict(self, cv_img=None, path=None):
        """
        predict 函数, 执行预测
        Keyword arguments::
        cv_img: image 输入opencv图像
        path: str 图片路径
        """
        with self.session.as_default():
            with self.graph.as_default():
                data_dir = os.path.join(self.data_root, self.ns)
                if path is not None:
                    cv_img = cv2.imread(path)
                label_names = get_labels(data_dir)
                rs_img_f32 = cv2.resize(
                    cv_img, (self.IMAGE_SIZE, self.IMAGE_SIZE)).astype('float32')
                input_data = rs_img_f32.reshape(-1,
                                                self.IMAGE_SIZE, self.IMAGE_SIZE, 3)
                if self.model is not None:
                    result = self.model.predict(input_data, steps=1)
                    print(label_names)
                    print("result:", result)
                    return label_names, result[0]
                else:
                    print('your model is not ready')
                    return None


def test_thread():
    """
    test_thread 函数, 多线程测试
    """
    import thread
    import time
    IC = ImageClassifier()

    def test():
        global IC
        with IC.graph.as_default():
            IC.predict(path='/root/keras/分类测试/分类1/1.jpg')
    try:
        IC.ns = '分类测试'
        # IC.load_model('/root/keras/分类测试/model.h5')
        IC.train(os.path.join(IC.data_root, '分类测试'), 2)
        # IC.predict(path='/root/keras/分类测试/分类1/1.jpg')
    except Exception as e:
        raise e
    # thread.start_new_thread(test,())
    # time.sleep(5)
    # train16()
    # train19()


def test_train():
    """
    test_train 函数, 训练测试
    """
    IC = ImageClassifier()
    IC.ns = '分类测试'

    def pub_training_logs(epoch, batch, logs):
        # logs {'loss': 0.33773628, 'accuracy': 0.71428573, 'batch': 6, 'size': 4}
        print(logs)
        msg = '第%d/%d轮, 批次: %d, 损失: %.2f, 准确率: %.2f' % (
            epoch+1, 2, batch, logs['loss'], logs['acc'])
        print(msg)
    IC.train(os.path.join(IC.data_root, '分类测试'), 2, callback=pub_training_logs)


if __name__ == '__main__':
    IC = ImageClassifier()
    IC.download_model()
    # test_train()
