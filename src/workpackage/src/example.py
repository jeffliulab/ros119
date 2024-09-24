import tensorflow as tf
import time
import matplotlib.pyplot as plt
import numpy as np

# 下载并加载MNIST数据集
mnist = tf.keras.datasets.mnist
(x_train, y_train), (x_test, y_test) = mnist.load_data()
x_train, x_test = x_train / 255.0, x_test / 255.0

# 定义简单的神经网络模型
model = tf.keras.models.Sequential([
    tf.keras.layers.Flatten(input_shape=(28, 28)),
    tf.keras.layers.Dense(128, activation='relu'),
    tf.keras.layers.Dropout(0.2),
    tf.keras.layers.Dense(10)
])

# 编译模型
model.compile(optimizer='adam',
              loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
              metrics=['accuracy'])

# 自定义回调函数用于记录每个 epoch 的时间
class TimeHistory(tf.keras.callbacks.Callback):
    def on_epoch_begin(self, epoch, logs=None):
        self.start_time = time.time()

    def on_epoch_end(self, epoch, logs=None):
        epoch_time = time.time() - self.start_time
        print(f"Epoch {epoch + 1} took {epoch_time:.2f} seconds")

# 训练模型并记录每个 epoch 的时间
time_callback = TimeHistory()

history = model.fit(x_train, y_train, epochs=5, callbacks=[time_callback])

# 评估模型
test_loss, test_acc = model.evaluate(x_test, y_test, verbose=2)
print(f'\nTest accuracy: {test_acc}')

# 使用模型进行预测
predictions = model.predict(x_test)

# 保存一些预测结果作为图像文件
for i in range(3):  # 保存前三个预测图像
    plt.imshow(x_test[i], cmap=plt.cm.binary)
    plt.title(f"Prediction: {np.argmax(predictions[i])}")
    plt.savefig(f'mnist_prediction_{i}.png')
    plt.close()

print("Prediction images saved.")
