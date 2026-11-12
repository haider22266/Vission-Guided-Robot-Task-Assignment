"""Exercise an actual GPU convolution before CDD training."""
import tensorflow as tf

devices=tf.config.list_physical_devices('GPU')
if not devices:
    raise RuntimeError('No TensorFlow GPU detected; check WSL/CUDA installation')
for device in devices:tf.config.experimental.set_memory_growth(device,True)
with tf.device('/GPU:0'):
    result=tf.nn.conv2d(tf.ones((1,64,64,3)),tf.ones((3,3,3,8)),strides=1,padding='VALID')
    print('TensorFlow',tf.__version__,'device',result.device,'convolution_sum',float(tf.reduce_sum(result)),flush=True)
